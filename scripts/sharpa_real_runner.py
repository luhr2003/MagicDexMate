#!/usr/bin/env python
"""Distributed real-hand receiver: ZMQ qpos stream -> real SharpaWave hand(s).

The receiving half of bridged teleop (the producer is scripts/teleop_retarget.py
--source wuji, which publishes 22-DoF qpos over ZMQ). One or both hands.

Single hand:
  # terminal 1 (teleop .venv): producer
  PYTHONPATH= .venv/bin/python scripts/teleop_retarget.py --source wuji --hand right
  # terminal 2: this receiver (SDK libs on LD_LIBRARY_PATH)
  LD_LIBRARY_PATH=/opt/sharpa-wave-sdk/lib PYTHONPATH= .venv/bin/python \
      scripts/sharpa_real_runner.py --hand right --sub tcp://127.0.0.1:5556

Both hands (two producers on two ports + one receiver):
  PYTHONPATH= .venv/bin/python scripts/teleop_retarget.py --source wuji --hand right --pub tcp://*:5556
  PYTHONPATH= .venv/bin/python scripts/teleop_retarget.py --source wuji --hand left  --pub tcp://*:5557
  LD_LIBRARY_PATH=/opt/sharpa-wave-sdk/lib PYTHONPATH= .venv/bin/python \
      scripts/sharpa_real_runner.py --hand both --sub tcp://127.0.0.1:5556 --sub-left tcp://127.0.0.1:5557

SAFETY: starts DISENGAGED. Keys while running: [e] engage  [w] freeze  [q] home+freeze
[x] stop & quit. Watchdog freezes a hand if its qpos stream goes stale. Joint
targets are limit-clipped and per-step rate-limited inside SharpaRealHand.
"""

from __future__ import annotations

import argparse
import json
import select
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from magicdexmate.sinks.sharpa_real import SharpaRealHand  # noqa: E402


class QposSub:
    """Non-blocking SUB of one teleop qpos stream; keeps the newest valid frame."""

    def __init__(self, addr: str, hand: str):
        import zmq

        self._zmq = zmq
        ctx = zmq.Context.instance()
        self.sock = ctx.socket(zmq.SUB)
        self.sock.connect(addr)
        self.sock.setsockopt_string(zmq.SUBSCRIBE, "")
        self.hand = hand
        self.crc = None
        self.qpos = None
        self.valid = False
        self.t_recv = 0.0  # local monotonic time of last fresh valid frame

    def poll(self):
        while True:
            try:
                msg = json.loads(self.sock.recv_string(flags=self._zmq.NOBLOCK))
            except self._zmq.Again:
                return
            if msg.get("hand") != self.hand:
                continue
            if msg["type"] == "hello":
                self.crc = msg["crc"]
            elif msg["type"] == "qpos":
                if self.crc is not None and msg["crc"] != self.crc:
                    continue  # producer restarted with a different joint order
                self.qpos = np.asarray(msg["qpos"], dtype=float)
                self.valid = bool(msg["valid"])
                if self.valid:
                    self.t_recv = time.monotonic()


class KeyPoller:
    """Non-blocking single-key reader from a terminal (no-op if stdin isn't a tty)."""

    def __enter__(self):
        self.tty = sys.stdin.isatty()
        if self.tty:
            import termios
            import tty as _tty

            self.fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(self.fd)
            _tty.setcbreak(self.fd)
        return self

    def get(self):
        if self.tty and select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def __exit__(self, *exc):
        if self.tty:
            import termios

            termios.tcsetattr(self.fd, termios.TCSADRAIN, self._old)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--hand", choices=["right", "left", "both"], default="right")
    p.add_argument("--sub", default="tcp://127.0.0.1:5556", help="qpos stream for right/single hand")
    p.add_argument("--sub-left", default="tcp://127.0.0.1:5557", help="qpos stream for the left hand (--hand both)")
    p.add_argument("--rate", type=float, default=20.0, help="control loop rate [Hz] (ramp up after it's stable)")
    p.add_argument("--stale-ms", type=float, default=150.0, help="freeze a hand if its stream is older than this")
    p.add_argument("--speed-coeff", type=float, default=0.3)
    p.add_argument("--current-coeff", type=float, default=0.3)
    p.add_argument("--max-step-rad", type=float, default=0.05, help="per-joint per-step cap (engage ramp + glitch guard)")
    p.add_argument("--auto-engage", action="store_true", help="engage at start without pressing 'e' (use with care)")
    return p.parse_args()


def main():
    args = parse_args()
    pairs = [("right", args.sub)] if args.hand == "right" else \
            [("left", args.sub)] if args.hand == "left" else \
            [("right", args.sub), ("left", args.sub_left)]

    hands, subs = [], []
    for hand, addr in pairs:
        rh = SharpaRealHand(hand, speed_coeff=args.speed_coeff, current_coeff=args.current_coeff,
                            max_step_rad=args.max_step_rad)
        rh.connect().configure()
        hands.append(rh)
        subs.append(QposSub(addr, hand))
        print(f"[runner] {hand} hand <- {addr}")

    if args.auto_engage:
        for rh in hands:
            rh.engage()
    print("[runner] keys: [e]ngage  [w] freeze  [q] home+freeze  [x] stop&quit   (starts DISENGAGED)")

    period = 1.0 / args.rate
    stale_s = args.stale_ms / 1000.0
    last_print = time.time()
    try:
        with KeyPoller() as keys:
            while True:
                t0 = time.perf_counter()
                key = keys.get()
                if key in ("e", "E"):
                    for rh in hands:
                        rh.engage()
                elif key in ("w", "W"):
                    for rh in hands:
                        rh.freeze()
                elif key in ("q", "Q"):
                    for rh in hands:
                        rh.freeze()
                        rh.go_home()
                elif key in ("x", "X"):
                    break

                now = time.monotonic()
                for rh, sub in zip(hands, subs):
                    sub.poll()
                    fault = rh.fault_code()
                    if fault != 0:
                        rh.freeze()
                        print(f"\n[runner] {rh.hand} FAULT 0x{fault:x} -> frozen")
                    fresh = sub.valid and (now - sub.t_recv) < stale_s
                    if fresh and sub.qpos is not None:
                        rh.command(sub.qpos)
                    elif rh._engaged and sub.t_recv > 0 and (now - sub.t_recv) >= stale_s:
                        rh.freeze()  # watchdog: stream went stale
                        print(f"\n[runner] {rh.hand} stream stale -> frozen (press 'e' to re-engage)")

                if time.time() - last_print > 2.0:
                    st = "  ".join(f"{rh.hand}:{'ENG' if rh._engaged else 'off'}" for rh in hands)
                    print(f"\r[runner] {st}", end="", flush=True)
                    last_print = time.time()

                sleep = period - (time.perf_counter() - t0)
                if sleep > 0:
                    time.sleep(sleep)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[runner] stopping")
        for rh in hands:
            rh.stop()


if __name__ == "__main__":
    main()

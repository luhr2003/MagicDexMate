"""Real SharpaWave hand control via the vendor SDK (`/opt/sharpa-wave-sdk`).

This is the receiving half of the bridged teleop: the producer
(`scripts/teleop_retarget.py --source wuji`) publishes 22-DoF qpos over ZMQ;
`scripts/sharpa_real_runner.py` consumes it and drives the real hand(s) through
this class. One `SharpaRealHand` == one physical hand (left or right).

Verified against SharpaWaveSDK 5.0.3 (sample/python + User Manual_250926):
  manager = SharpaWaveManager.get_instance(); time.sleep(1)   # device discovery
  hand = manager.connect(HandSide.RIGHT)        # or connect(sn); throws if 0/>1 match
  hand.set_control_mode(ControlMode.POSITION)   # every set_* returns Error(.code,.message)
  hand.set_speed_coeff(0.3); hand.set_current_coeff(0.3)
  hand.set_control_source(ControlSource.SDK); hand.start()
  hand.set_joint_position(q22_rad, interpolate)  # SDK order == sharpa_sdk_joint_names
  err, ang = hand.get_joint_position_rad()
  hand.stop(); SharpaWaveManager.get_instance().disconnect_all()

SDK joint order (Manual 1.1) == magicdexmate.retarget.mapping.sharpa_sdk_joint_names.
Units are radians. Firmware auto-interpolates any >20 deg per-frame jump; we keep
each host step well under that with an explicit rate limit, so streaming stays
direct/responsive while the engage ramp and any glitch are bounded.
"""

from __future__ import annotations

import sys

import numpy as np

SDK_PYTHON = "/opt/sharpa-wave-sdk/python"
SDK_LIB = "/opt/sharpa-wave-sdk/lib"  # export LD_LIBRARY_PATH=$SDK_LIB before launch if libs don't resolve

# (lower, upper) rad in SDK / URDF order; identical for left and right (verified).
SHARPA_JOINT_LIMITS = np.array([
    (-0.1745, 1.9199),  # thumb_CMC_FE
    (-0.3491, 0.3491),  # thumb_CMC_AA
    (-0.5236, 1.3963),  # thumb_MCP_FE
    (-0.3491, 0.3491),  # thumb_MCP_AA
    (0.0000, 1.7453),   # thumb_IP
    (-0.1745, 1.5708),  # index_MCP_FE
    (-0.3491, 0.3491),  # index_MCP_AA
    (0.0000, 1.7453),   # index_PIP
    (0.0000, 1.3963),   # index_DIP
    (-0.1745, 1.5708),  # middle_MCP_FE
    (-0.3491, 0.3491),  # middle_MCP_AA
    (0.0000, 1.7453),   # middle_PIP
    (0.0000, 1.3963),   # middle_DIP
    (-0.1745, 1.5708),  # ring_MCP_FE
    (-0.3491, 0.3491),  # ring_MCP_AA
    (0.0000, 1.7453),   # ring_PIP
    (0.0000, 1.3963),   # ring_DIP
    (0.0000, 0.2618),   # pinky_CMC
    (-0.1745, 1.5708),  # pinky_MCP_FE
    (-0.3491, 0.3491),  # pinky_MCP_AA
    (0.0000, 1.7453),   # pinky_PIP
    (0.0000, 1.3963),   # pinky_DIP
])


def _import_sdk():
    if SDK_PYTHON not in sys.path:
        sys.path.insert(0, SDK_PYTHON)
    import sharpa  # noqa: E402
    return sharpa


class SharpaRealHand:
    """Safe POSITION-mode driver for one physical SharpaWave hand.

    Safety chain (all on by default): scaled joint-limit clip, per-step rate
    limit (bounds engage ramp and any glitch), collision protection, conservative
    speed/current coeffs, fault-code watchdog. Starts DISENGAGED - call engage()
    before commands move the hand.
    """

    def __init__(
        self,
        hand: str,
        sn: str | None = None,
        speed_coeff: float = 0.3,
        current_coeff: float = 0.3,
        limit_scale: float = 0.9,
        max_step_rad: float = 0.05,
    ):
        assert hand in ("left", "right")
        self.hand = hand
        self.sn = sn
        self.speed_coeff = speed_coeff
        self.current_coeff = current_coeff
        center = SHARPA_JOINT_LIMITS.mean(axis=1)
        half = (SHARPA_JOINT_LIMITS[:, 1] - SHARPA_JOINT_LIMITS[:, 0]) / 2.0 * limit_scale
        self.lo = center - half
        self.hi = center + half
        self.max_step = float(max_step_rad)

        self._sdk = None
        self._mgr = None
        self._hand = None
        self._engaged = False
        self._last_cmd: np.ndarray | None = None  # last sent target (rad, SDK order)

    # -- lifecycle ------------------------------------------------------------

    def _check(self, err, what: str):
        if err is not None and getattr(err, "code", 0) != 0:
            raise RuntimeError(f"[sharpa:{self.hand}] {what} failed: {err.message} (code {err.code})")

    def connect(self, discovery_wait_s: float = 1.5):
        import time

        self._sdk = _import_sdk()
        s = self._sdk
        self._mgr = s.SharpaWaveManager.get_instance()
        time.sleep(discovery_wait_s)  # let heartbeat discovery populate
        if self.sn is not None:
            self._hand = self._mgr.connect(self.sn)
        else:
            side = s.HandSide.LEFT if self.hand == "left" else s.HandSide.RIGHT
            self._hand = self._mgr.connect(side)  # throws if 0 or >1 matching hands
        info = self._hand.get_device_info()
        if str(s.DeviceType(info.device_type).name) != "HAND":
            raise RuntimeError(f"[sharpa:{self.hand}] connected device is not a HAND: {info.device_type}")
        got_side = "left" if info.hand_side == s.HandSide.LEFT else "right"
        if got_side != self.hand:
            raise RuntimeError(f"[sharpa] asked for {self.hand} but connected {got_side} (sn={info.sn})")
        print(f"[sharpa:{self.hand}] connected sn={info.sn} ip={info.ip} fw={info.firmware_version}")
        return self

    def configure(self):
        s, h = self._sdk, self._hand
        self._check(h.set_control_mode(s.ControlMode.POSITION), "set_control_mode")
        self._check(h.set_speed_coeff(self.speed_coeff), "set_speed_coeff")
        self._check(h.set_current_coeff(self.current_coeff), "set_current_coeff")
        self._check(h.set_control_source(s.ControlSource.SDK), "set_control_source")
        try:
            h.enable_collision_protection()
        except Exception as e:  # noqa: BLE001 - not all firmware exposes it
            print(f"[sharpa:{self.hand}] collision protection unavailable: {e}")
        self._check(h.start(), "start")
        print(f"[sharpa:{self.hand}] configured POSITION speed={self.speed_coeff} current={self.current_coeff}")
        return self

    # -- control --------------------------------------------------------------

    def current_angles(self) -> np.ndarray:
        err, ang = self._hand.get_joint_position_rad()
        self._check(err, "get_joint_position_rad")
        return np.asarray(ang, dtype=float)

    def engage(self):
        """Arm the hand: seed the rate limiter at the current pose so the first
        commands ramp smoothly from where the hand actually is (no jump)."""
        self._last_cmd = self.current_angles()
        self._engaged = True
        print(f"[sharpa:{self.hand}] ENGAGED (ramping from current pose)")

    def freeze(self):
        """Hold the current target; ignore incoming commands until re-engaged."""
        self._engaged = False
        print(f"[sharpa:{self.hand}] FROZEN")

    def command(self, q_sdk: np.ndarray) -> bool:
        """Drive toward q_sdk (22 rad, SDK order) with clip + per-step rate limit.

        No-op (returns False) unless engaged. Direct (non-interpolated) sends;
        the rate limit keeps every step < the firmware's 20 deg auto-interp
        threshold, so motion is responsive yet bounded.
        """
        if not self._engaged:
            return False
        target = np.clip(np.asarray(q_sdk, dtype=float), self.lo, self.hi)
        if self._last_cmd is None:
            self._last_cmd = self.current_angles()
        step = np.clip(target - self._last_cmd, -self.max_step, self.max_step)
        self._last_cmd = self._last_cmd + step
        self._check(self._hand.set_joint_position(self._last_cmd.tolist(), False), "set_joint_position")
        return True

    def go_home(self, blocking_interp: bool = True):
        """Move to the all-zero pose (firmware interpolates smoothly)."""
        self._check(self._hand.set_joint_position([0.0] * 22, blocking_interp), "set_joint_position(home)")
        self._last_cmd = np.zeros(22)

    def fault_code(self) -> int:
        try:
            return int(self._hand.get_fault_code())
        except Exception:
            return 0

    def stop(self):
        self._engaged = False
        if self._hand is not None:
            try:
                self._hand.stop()
            except Exception:
                pass
        if self._mgr is not None:
            try:
                self._mgr.disconnect_all()
            except Exception:
                pass
        print(f"[sharpa:{self.hand}] stopped + disconnected")

    def __enter__(self):
        return self.connect().configure()

    def __exit__(self, *exc):
        self.stop()
        return False

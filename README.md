# MagicDexMate

Dexmate Vega + SharpaWave 灵巧手遥操作（real + sim）：

> **Wuji 数据手套**（21 关键点 @120Hz，MediaPipe 格式）→ **dex-retargeting**（vector/DexPilot 优化）→ **SharpaWave 22 DOF 关节角** → Isaac Lab 仿真 / SharpaWave 真机；后续接入 Dexmate Vega 手臂遥操作（Pico/VR）与 SLAM（见 plans/00_roadmap.md）。

> 📄 retarget 链路的专门说明见 **[magicdexmate/retarget/README.md](magicdexmate/retarget/README.md)**。

## 两种运行模式

```
模式 A（双进程桥接，默认；真机也走这条路）
  [teleop 进程 .venv]  手套(wuji|mock) → retarget → ZMQ PUB(:5556, JSON+CRC)
  [Isaac Lab 进程]     sim/test_env_sharpa.py --motion zmq  ← SUB 跟随
  [真机进程 P3]        sinks/sharpa_real.py（计划 03）      ← SUB 跟随

模式 B（单进程，全部塞进 Isaac Lab python）
  [Isaac Lab 进程]     sim/teleop_isaac_single.py：手套 + retarget + 仿真，无 ZMQ
```

模式 A 的优点：一份 qpos 流同时喂 sim/real（天然 A/B 对照）、崩溃隔离、teleop 侧可用最新依赖。模式 B 的优点：少开一个终端、env 内直接拿 retarget 结果（采数据方便）。
**numpy 说明**：dex-retargeting 声明 `numpy>=2.0`，但源码无 numpy-2 专属 API——已实证在 Isaac 的 numpy 1.26 上跑出**逐位一致**的结果（`--no-deps` 安装即可，脚本已处理；详见 docs/references/03 §6），所以两种模式都成立。

## 目录结构

```
magicdexmate/            # 核心包（teleop venv 与 Isaac python 通用）
  skeleton.py            # MediaPipe 21 点约定、HandFrame、名字→索引
  sources/               # wuji_source.py（真手套，按 wuji-sdk 2026.6.2 实测 API）
                         # mock_source.py（合成手：open/fist/wave/pinch/cycle）
  retarget/              # frames.py（逐帧 estimate_frame→MANO，对手套腕系免疫）
                         # builder.py（构建 SeqRetargeting）mapping.py（按名映射 SDK 22 序+限位 clip）
  sinks/                 # qpos_publisher.py（ZMQ，hello 带 joint_names+CRC）sapien_viz.py（调试视图）
configs/retargeting/     # sharpa_wave_{right,left}_{vector,dexpilot}.yml（scaling 1.07 实测）
assets/robots/hands/     # prepare_assets.py 生成的 Sharpa URDF+mesh（路径已修正）
sim/                     # sharpa_scene.py（共享场景，参数照抄 sharpa-rl-lab）
                         # test_env_sharpa.py（模式 A 消费端；sine/home/zmq）
                         # teleop_isaac_single.py（模式 B 单进程，裸手）
                         # vega_sharpa_scene.py + teleop_vega_sharpa.py（Vega-1P 整机，
                         #   资产/增益取自 MagicSim，腕部 rpy 姿态 IK）
scripts/                 # setup_env.sh / install_into_isaaclab.sh / prepare_assets.py
                         # check_env.py（自检）/ teleop_retarget.py（模式 A 主程序）
tests/                   # pytest（11 项）
plans/  docs/references/ # 计划与参考笔记（见文末导航）
```

## 环境安装

### 1. teleop 环境（uv，已建好在 `.venv/`）

```bash
bash scripts/setup_env.sh      # uv venv py3.11 + torch-cpu + dex-retargeting(-e) + wuji-sdk + sapien + 自检
```

### 2. Isaac 环境（uv venv，`.venv-isaac/`）

isaacsim + isaaclab 以 pip wheel 形式装进独立 uv 环境（`isaaclab[isaacsim]==2.3.0` 自动带配套 isaacsim；retarget 栈同环境内、numpy 受约束保护）：

```bash
bash scripts/setup_isaac_env.sh        # 首次约 GB 级下载（pypi.nvidia.com）
```

首次启动需接受 Omniverse EULA：命令前加 `OMNI_KIT_ACCEPT_EULA=YES`。
（备选：若想复用 `~/isaacsim` standalone 安装，可用 `scripts/install_into_isaaclab.sh ~/isaacsim/python.sh` 把 retarget 栈装进它自带的 python——但其老 pip 解析很慢，推荐 uv 路线。）

### 3. Wuji 手套硬件

主机网卡设 192.168.1.x/24；左手 192.168.1.100、右手 .101（UDP 50000/50001）。标定用 **Wuji Studio**（已下载到 `~/Downloads/`）：

```bash
sudo apt install ~/Downloads/wuji-studio_2026.6.2_amd64.deb   # 然后运行 wuji-studio
```

## 快速开始

```bash
# 自检（teleop venv）：imports/资产/4配置构建/跟踪精度/mock管线/ZMQ/耗时 —— 10/10
.venv/bin/python scripts/check_env.py
.venv/bin/python -m pytest tests/ -q                          # 11/11

# ---- 模式 A：双进程桥接 ----
.venv/bin/python scripts/teleop_retarget.py --source mock --motion cycle --viz   # 终端1
.venv-isaac/bin/python sim/test_env_sharpa.py --motion zmq                       # 终端2
.venv-isaac/bin/python sim/test_env_sharpa.py --motion sine                      # 只测场景/资产/执行器

# ---- 模式 B：单进程（全在 .venv-isaac）----
.venv-isaac/bin/python sim/teleop_isaac_single.py --source mock --motion cycle
.venv-isaac/bin/python sim/teleop_isaac_single.py --source mock --headless --duration 10  # 无显示冒烟

# ---- Vega-1P + Sharpa 整机（单进程；手指=retarget，腕=手套 rpy 姿态 IK）----
# 资产/增益来自 MagicSim（Assets/Robots/vega_1p_sharpa.usd + vega1psharpa.py 的实测参数）
OMNI_KIT_ACCEPT_EULA=YES .venv-isaac/bin/python sim/teleop_vega_sharpa.py --source mock
OMNI_KIT_ACCEPT_EULA=YES .venv-isaac/bin/python sim/teleop_vega_sharpa.py --source wuji --hand right
# 说明：Wuji 手套只有腕部姿态（IMU rpy），没有 translation —— 腕控用 DifferentialIK
# 做"姿态跟踪"：位置目标钉在 R_ee 初始位置，姿态=初始姿态⊗手套相对旋转；
# P4 接入 Pico 后只需把位置目标换成 VR 轨迹，控制结构不变。--wrist off 可关腕控。

# ---- 真手套（到货后把 mock 换掉即可）----
.venv/bin/python scripts/teleop_retarget.py --source wuji --hand right --viz             # 模式 A
.venv-isaac/bin/python sim/teleop_isaac_single.py --source wuji --hand right             # 模式 B
```

常用参数：`--hand right|left`、`--mode vector|dexpilot`（捏合任务用 dexpilot）、`--rate/--control_hz`（默认 60）、`--sn/--address`（指定手套）、`--duration N`（定时退出，冒烟用）、`--no-pub`。

## 实测指标（2026-06-11，mock 全链路）

| 指标 | 数值 |
|---|---|
| 指尖向量跟踪误差（FK 验证，60Hz 顺序跟踪） | 四指 7-11mm，拇指 ~17mm |
| retarget 单步耗时 | p50 2.7ms / p95 5.0ms（≈200Hz 上限；numpy1.26+pin2.7 组合实测 2.0ms） |
| 端到端（采集→ZMQ 发布） | p50 6.4ms @ 60Hz |
| 自检 / 单测 | check_env 10/10，pytest 11/11 |

## 已知问题与注意

1. **随机极端位姿单步求解差**（~52mm 驻点；梯度已验证正确，nlopt 2.8/2.10 一致）：teleop 连续小步跟踪不受影响；离线批处理勿从任意位姿单步求解。详见 docs/references/03 §5.5。
2. 拇指跟踪偏松：mock 拇指几何粗糙 + 人/机拇指构型差异，真手套数据到位后按计划 01 R5 调 scaling/DexPilot 参数。
3. `sim/*.py` 用 `.venv-isaac`（isaacsim+isaaclab+retarget 栈），`scripts/*.py` 用 `.venv`；两个环境都由 uv 管理。Isaac 首次启动加 `OMNI_KIT_ACCEPT_EULA=YES`。
4. **isaacsim 退出会挂死**（`simulation_app.close()` 不返回）：三个 sim 脚本都已内置 20 秒退出看门狗（数据落盘后 `os._exit(0)` 硬退），无需手工 pkill。另：本机 CPU governor=powersave，仿真显著慢于墙钟（Kit 启动时有警告），跑 headless 冒烟请给大 timeout 或 `sudo cpupower frequency-set -g performance`。
5. **headless 自检拍照**：`sim/teleop_vega_sharpa.py --snap-dir DIR --snap-times "0.5,4,15.5" --cam-eye "1.6,-1.3,1.7" --cam-target "0.0,-0.2,1.15"`（自动启用离屏渲染），可在无显示器环境拍 RGB 截图核对动作；`--debug-joints` 输出分组关节遥测。
6. **vega_1p_sharpa.usd 的雅可比行索引**：该 USD 的 ArticulationRootAPI 在 /root_joint 上，DiffIK 取雅可比应使用 `body_idx` 行（脚本默认 `--jacobi-row body`，实测保持误差 0-2mm；教程式 `body_idx-1` 会 200-600mm 跑飞）。
7. GPU 与其他任务并发压力下，Kit 图形上下文可能 Xid 31 MMU fault 并**静默 exit 0**（无 traceback；`journalctl -k | grep -i xid` 可查）——遇到"无输出正常退出"先查这个和显存占用。
4. **Sharpa 厂商资料（4 份 PDF + SDK 安装包）在本机全为 0 字节**，真机阶段（P3）前需重新拷贝——参考笔记 02 是从 sharpa-rl-lab 部署代码反推的替代资料。
5. docs.dexmate.ai 需客户账号（401）；omniteleop 的 VR 代码面向 Meta Quest，**PICO 兼容性待确认**（P4 阻塞项）。

## 文档导航

| 文档 | 内容 |
|---|---|
| [plans/00_roadmap.md](plans/00_roadmap.md) | 总路线图 P0-P6、架构、决策记录 |
| [plans/01_retarget_plan.md](plans/01_retarget_plan.md) | retarget 计划（含实施状态：R0/R1/R4/R6 已完成）|
| [plans/02_sim_scene_plan.md](plans/02_sim_scene_plan.md) | Isaac 场景：浮动基座、双手、Vega 装配、触觉 |
| [plans/03_sharpa_real_teleop_plan.md](plans/03_sharpa_real_teleop_plan.md) | 真机操控：bring-up、安全链、联调步骤 |
| [docs/references/01_wuji_glove.md](docs/references/01_wuji_glove.md) | Wuji 手套硬件/SDK/数据流/标定 |
| [docs/references/02_sharpa_wave_hand.md](docs/references/02_sharpa_wave_hand.md) | SharpaWave 关节表/SDK API/触觉 180Hz |
| [docs/references/03_dex_retargeting.md](docs/references/03_dex_retargeting.md) | dex-retargeting 用法 + 实测结论/已知问题 |
| [docs/references/04_dexmate_vega.md](docs/references/04_dexmate_vega.md) | Vega/dexcontrol/omniteleop/SLAM |
| [docs/references/05_sharpa_sim_assets.md](docs/references/05_sharpa_sim_assets.md) | URDF/USD/MJCF 资产变体 + sharpa-rl-lab 解析 |

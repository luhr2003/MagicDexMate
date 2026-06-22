# SharpaWave 灵巧手参考笔记（硬件 + SDK + 触觉）

> ⚠ **重要：4 份官方 PDF（SharpaWave / SharpaWaveSDK / SharpaPilot / 180Hz Tactile）在 `/home/msc/luhr/magicsim/MagicTactile/UserManual/` 下全是 0 字节空文件**；`/home/msc/sharpa/` 与 `/media/msc/5TB/sharpa/` 下整个 SDK 树（493 个文件，含 SharpaWaveSDK_4.6.6 的 .so、python 包、样例、`sharpa-pilot_1_2_27_amd64_linux.deb`）也全为空，疑似 rsync 只同步了元数据。**需要从原始来源重新拷贝。**
> 本文内容是从可用代码反推的：`sharpa-rl-lab/rl_isaaclab/tasks/inhand_rotate/sharpa_wave_deploy_env.py`（已验证可运行的 SDK 用法）、`sharpa-urdf-usd-xml` URDF、SDK 目录结构与 Doxygen 文件名。整理日期 2026-06-11。

## 1. 硬件概要

- 5 指，**22 个主动旋转关节**（无 mimic/耦合关节），左右手版本齐全；硬件代号 **ha4**。
- 指尖触觉：**5 个视觉触觉传感器（VBTS）**，每指尖一个，位于 elastomer 上。
- 通信：**网络接入**（`DeviceInfo` 有 `.ip` 字段，SDK 走 UDP，docker 需 `network_mode: host`）。接插件/供电规格只在空 PDF 里，未知。
- 控制模式：**POSITION**（默认，仿真动力学按此整定）与 **MIT 模式**（力控/PD，URDF 仓库的 `*_MITmode.usda`；README 建议 MIT 模式控制频率 **>80 Hz**）。
- 单位一律 **rad**；home 位 = 22 个 0。
- 法兰安装：URDF 的 `with_flange` 变体以法兰为根 link，用于装在机械臂末端。

## 2. 关节表（右手，URDF 顺序 = SDK `set_joint_position` 顺序）

根 link：`right_hand_C_MC`（左手 `left_hand_C_MC`）。limits 单位 rad，effort 单位 N·m。

| # | 关节 | lower | upper | effort |
|---|---|---|---|---|
| 0 | right_thumb_CMC_FE | -0.1745 | 1.9199 | 3.3 |
| 1 | right_thumb_CMC_AA | -0.3491 | 0.3491 | 3.3 |
| 2 | right_thumb_MCP_FE | -0.5236 | 1.3963 | 1.864 |
| 3 | right_thumb_MCP_AA | -0.3491 | 0.3491 | 1.864 |
| 4 | right_thumb_IP | 0 | 1.7453 | 0.638 |
| 5–8 | right_index_{MCP_FE, MCP_AA, PIP, DIP} | -0.1745 / -0.3491 / 0 / 0 | 1.5708 / 0.3491 / 1.7453 / 1.3963 | 1.864 / 1.864 / 0.638 / 0.189 |
| 9–12 | right_middle_{MCP_FE, MCP_AA, PIP, DIP} | 同 index | 同 index | 同 index |
| 13–16 | right_ring_{MCP_FE, MCP_AA, PIP, DIP} | 同 index | 同 index | 同 index |
| 17 | right_pinky_CMC | 0 | 0.2618 | 0.5285 |
| 18–21 | right_pinky_{MCP_FE, MCP_AA, PIP, DIP} | 同 index | 同 index | 同 index |

指尖相关 link（每指 3 个坐标系，经 fixed joint 挂接）：
- `*_DP`：远端指节（MDH 约定）
- `*_elastomer`：触觉传感器系
- `*_fingertip`：**IK 指尖标注**（retargeting 用这个）→ `right_{thumb,index,middle,ring,pinky}_fingertip`

## 3. SDK（Python，模块名 `sharpa`）

- 安装：把 SDK 的 `python/sharpa/` 目录放进 `sys.path`（无 pip 包）；预编译 `sharpa.so` 提供 python **3.10–3.13**。C++ 侧命名空间 `sharpa`（类 `SharpaWave`、`SharpaWaveManager`、`tactile::DataBlock` 等）。
- ROS 支持 = 样例脚本 `sample/ROS/{wave_ros_server.py, wave_ros_client.py}`。
- 所有控制调用返回 `Error`（`.code`==0 为成功，`.message`）。

已验证的用法（摘自 sharpa_wave_deploy_env.py:129–193）：

```python
from sharpa import SharpaWaveManager, ControlMode, ControlSource
manager = SharpaWaveManager.get_instance(); time.sleep(1)     # 等待设备发现
sns = manager.get_all_device_sn()
hand = manager.connect(sns[0])
info = hand.get_device_info()                                  # 有 .ip

hand.set_control_mode(ControlMode.POSITION)
hand.set_speed_coeff(0.3)        # 0–1，速度系数（部署代码用 0.3/0.5）
hand.set_current_coeff(0.3)      # 电流限制系数
hand.set_control_source(ControlSource.SDK)
hand.set_tactile_config_file("~/.sharpa-pilot/config/tactile.json")   # 可选，触觉
hand.set_tactile_callback(cb)                                          # 可选
hand.start()

hand.set_joint_position([0.0]*22)        # rad，顺序=§2 表
angles = hand.get_states().angles        # 22 个 rad
hand.calib_tactile()                     # 触觉零位
hand.stop(); SharpaWaveManager.get_instance().disconnect_all()
```

- 限位：部署代码用 URDF 限位 × **0.9** 安全系数；注释称硬件取"该限位 ∩ 固件限位"。
- 参考部署控制频率：20 Hz（sleep 实现，非 SDK 上限）。
- 关节序与 Isaac Lab 顺序不同：换序写法参考 `sharpa-rl-lab/rl_isaaclab/utils/misc.py` 的 `_ISAACLAB2SHARPA_IDX / _SHARPA2ISAACLAB_IDX`。

## 4. 触觉系统

- 回调帧（dict）：`frame['channel']`（int；**右手=通道 0–4，左手=5–9**；部署代码内按 `4-ch` 读 ⇒ 通道号与手指的对应需上真机验证）；`frame['content']` 键：
  - `'RAW'`：触觉图像
  - `'F6'`：**6 轴力/力矩，前 3 个 = 力向量（N）**
  - `'CONTACT_POINT'`：N×3（像素 u、像素 v、深度）→ 经 240×240 点位图映射到 3D（mm，/1000 → m）
- 几何映射表：拇指一张（TH），四指共用一张（4F）：`tactileSensor_map_{TH,4F}_{point,normal}.npy`，**(240,240,3) float32，单位 mm**（在 `sharpa-rl-lab/assets/tactile_ha4_map/`）。
- 帧率：**板载推理 30 Hz；主机 GPU 推理 180 Hz**。
- 180Hz 配方（自 docker 配置反推，原 PDF 为空）：
  1. 装 docker + nvidia-container-toolkit，主机需 NVIDIA GPU；
  2. 跑厂商容器 `sharpadev/sharpawave-rl-deploy:1.0.2-cu124`（基底 `sharpadev/sharpawavesdk:4.3.2`），`runtime: nvidia`、`network_mode: host`、`privileged: true`、挂 X11（compose 见 `sharpa-rl-lab/rl_isaaclab/utils/docker-compose.yml`）；
  3. 改 `~/.sharpa-pilot/config/tactile.json`：`cuda.<left|right>.fps = 180`、`infer_from_device = false`（板载模式则 30/true）；
  4. `hand.set_tactile_config_file(...)` 后再 `hand.start()`，触觉帧经 callback 以 180Hz 到达。

## 5. SharpaPilot

- 桌面程序（deb 包，v1.2.27 amd64），作用：**手与触觉标定**（sharpa-rl-lab README §4.1 "Calibrate SharpaWave through SharpaPilot"），并维护运行时配置目录 `~/.sharpa-pilot/config/tactile.json`（docker 内 `/root/.sharpa-pilot/...`），SDK 用 `set_tactile_config_file` 消费它。
- 是否带 teleop/手套/VR 功能：未知（只在空 PDF 里）。本机所有文件无 "wuji" 字样。

## 6. 待确认清单（需补齐 PDF 后核对）

- 电气接口/连接器/供电规格；SDK 上限控制频率；MIT 模式 API；SharpaPilot 完整功能；触觉通道→手指的精确对应；触觉原始图像分辨率。

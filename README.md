# Face Robot — 人脸机器人软件栈

> 基于 Ubuntu 22.04 + ROS 2 Humble，驱动 POM SC32 32路舵机控制器，通过眼部舵机实现人脸追踪与自然眼部动作。

---

## 目录

- [项目架构](#项目架构)
- [软件层次图](#软件层次图)
- [ROS 2 接口总览](#ros-2-接口总览)
- [环境准备](#环境准备)
- [编译](#编译)
- [独立测试（不依赖 ROS）](#独立测试不依赖-ros)
- [启动 ROS 2 节点](#启动-ros-2-节点)
- [配置文件说明](#配置文件说明)
- [硬件协议速查](#硬件协议速查)
- [里程碑进度](#里程碑进度)

---

## 项目架构

```
face_robot_ws/
├── config/
│   └── servos.yaml             # 舵机参数：限位 / 方向 / 中位
├── models/
│   └── face_landmarker.task    # MediaPipe FaceLandmarker 模型
├── src/
│   └── face_robot_driver/      # 核心 ROS 2 包（ament_python）
│       ├── face_robot_driver/
│       │   ├── pom_sc32.py           # 协议层：串口通信 & HEX 指令封装
│       │   ├── servo_config.py       # 软限位：加载 servos.yaml，clamp/neutral
│       │   ├── driver_node.py        # ROS 2 驱动节点：接收角度指令 → 串口下发
│       │   ├── eye_controller.py     # 眼部逻辑：凝视/眨眼/眼睑耦合/自动闲置
│       │   ├── eye_node.py           # ROS 2 眼部节点：手动调试用
│       │   ├── face_tracker_node.py  # ROS 2 追踪节点：Haar 级联 → 眼球追踪
│       │   ├── expressions.py        # 共享表情预设（happy/angry/sad）
│       │   ├── emotion_detector.py   # 情绪检测：FaceLandmarker + 分类 + 滤波
│       │   └── emotion_mirror_node.py # ROS 2 情绪镜像节点：识别情绪 → 表情跟随
│       ├── scripts/
│       │   ├── test_servo.py         # 舵机独立测试（不依赖 ROS）
│       │   ├── test_eye.py           # 眼部独立测试（不依赖 ROS）
│       │   └── test_expression.py    # 表情独立测试（不依赖 ROS）
│       ├── package.xml
│       └── setup.py
└── README.md
```

---

## 软件层次图

```
【情绪镜像模式】（当前主要模式）

┌───────────────────────────────────┐
│       emotion_mirror_node         │  摄像头 + FaceLandmarker
│  blendshapes → 情绪分类 → EMA滤波  │  happy/angry/sad/neutral
│  smoothstep 过渡 → 表情舵机角度     │  舵机 1-8, 15-22, 28-30
└──────────────┬────────────────────┘
               │  /face/servo_angles  (Float32MultiArray, 32×float)
               ▼
┌───────────────────────────────────┐
│           driver_node             │  50 Hz 控制循环
│  watchdog / 软限位 / 急停          │  超时自动回 neutral
└──────────────┬────────────────────┘
               │  HEX 指令（65字节，USB-Serial 460800 8N1）
               ▼
┌───────────────────────────────────┐
│         POM SC32 驱动板            │  32路 PWM 舵机输出
└───────────────────────────────────┘

【眼球追踪模式】

face_tracker_node ──/face/servo_angles──→ driver_node → 硬件
  (Haar 级联 + 比例控制器，控制眼部舵机 9-14)

【手动调试模式】

eye_node ──/face/servo_angles──→ driver_node → 硬件
```

---

## ROS 2 接口总览

### 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/face/servo_angles` | `Float32MultiArray` | 32路角度(°)，索引0对应舵机1 |
| `/face/detected_emotion` | `String` | 当前识别情绪 (happy/angry/sad/neutral)，emotion_mirror 发布 |
| `/face/eye/gaze_cmd` | `Float32MultiArray` | `[pitch, yaw]` 或 `[pitch, yaw, duration_ms]`，eye_node 订阅 |

### 服务

| 服务 | 节点 | 说明 |
|------|------|------|
| `/face/estop` | driver_node | 急停：立即回中，拒绝后续指令 |
| `/face/resume` | driver_node | 解除急停 |
| `/face/eye/blink` | eye_node | 触发一次眨眼 |
| `/face/eye/wink` | eye_node | 右眼 wink |
| `/face/eye/center` | eye_node | 眼球回正中 |
| `/face/eye/idle_on/off` | eye_node | 开关自动闲置模式 |

### 节点参数

**driver_node**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `port` | `/dev/ttyUSB0` | 串口设备路径 |
| `baudrate` | `460800` | 波特率 |
| `watchdog_timeout` | `1.0` | 掉线超时(s)，超时后回 neutral |
| `servos_yaml` | `~/Mark/face_robot_ws/config/servos.yaml` | 舵机参数路径 |

**face_tracker_node**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `camera_index` | `0` | 摄像头编号 |
| `kp` | `0.08` | 比例增益，越大追踪越激进 |
| `deadzone_px` | `30` | 像素死区，小于此误差不响应 |
| `idle_timeout` | `3.0` | 无人脸后进入闲置模式的秒数 |
| `show_preview` | `false` | 显示摄像头检测预览窗口 |
| `servos_yaml` | `~/Mark/face_robot_ws/config/servos.yaml` | 舵机参数路径 |

**emotion_mirror_node**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `camera_index` | `0` | 摄像头编号 |
| `model_path` | `~/Mark/face_robot_ws/models/face_landmarker.task` | FaceLandmarker 模型路径 |
| `ema_alpha` | `0.3` | EMA 平滑系数，越小越稳定 |
| `min_hold_s` | `0.5` | 情绪切换最小间隔(s) |
| `confidence_threshold` | `0.25` | 切换所需分数差 |
| `transition_duration` | `0.6` | 表情过渡时长(s) |
| `show_preview` | `true` | 显示预览窗口（含情绪分数条） |
| `servos_yaml` | `~/Mark/face_robot_ws/config/servos.yaml` | 舵机参数路径 |

---

## 环境准备

```bash
# Python 依赖
pip3 install pyserial pyyaml mediapipe "numpy<2"

# 串口权限（需重新登录生效，或用 newgrp dialout 立即生效）
sudo usermod -a -G dialout $USER

# 如 /dev/ttyUSB0 不出现（Ubuntu 22.04 brltty 冲突）
sudo apt remove brltty
```

---

## 编译

```bash
cd ~/Mark/face_robot_ws
source /opt/ros/humble/setup.bash

# 首次编译
colcon build --symlink-install

# 加载环境（每个新终端执行一次）
source ~/Mark/face_robot_ws/install/setup.bash
```

---

## 独立测试（不依赖 ROS）

所有命令在 `~/Mark/face_robot_ws` 目录下执行。

### 舵机通信测试

```bash
# 单舵机扫描（默认：舵机1 在 min~max 间往复，验证通信链路）
python3 src/face_robot_driver/scripts/test_servo.py --port /dev/ttyUSB0

# 指定舵机到指定角度（保持直到 Ctrl+C）
python3 src/face_robot_driver/scripts/test_servo.py --port /dev/ttyUSB0 --servo 1 --angle 90

# 全部舵机回 neutral（servos.yaml 中各舵机的 neutral 值）
python3 src/face_robot_driver/scripts/test_servo.py --port /dev/ttyUSB0 --all
```

### 眼部动作测试

```bash
# 初始化（眼睛张开居中）
python3 src/face_robot_driver/scripts/test_eye.py --port /dev/ttyUSB0 --center

# 单次眨眼
python3 src/face_robot_driver/scripts/test_eye.py --port /dev/ttyUSB0 --blink

# 单眼 wink（right / left）
python3 src/face_robot_driver/scripts/test_eye.py --port /dev/ttyUSB0 --wink right

# 眼球凝视  pitch:-1(下)~+1(上)  yaw:-1(右)~+1(左)（Ctrl+C 退出回中）
python3 src/face_robot_driver/scripts/test_eye.py --port /dev/ttyUSB0 --gaze -0.5 0
python3 src/face_robot_driver/scripts/test_eye.py --port /dev/ttyUSB0 --gaze 0 0.5

# 自动闲置模式（随机扫视 + 周期眨眼，Ctrl+C 退出）
python3 src/face_robot_driver/scripts/test_eye.py --port /dev/ttyUSB0 --idle
```

**建议测试顺序：** `--center` → `--blink` → `--gaze -0.5 0`（观察眼睑耦合）→ `--idle`

### 表情动作测试

```bash
# 顺序展示：开心 → 正常 → 生气 → 正常 → 伤心 → 正常（每个 5 秒）
python3 src/face_robot_driver/scripts/test_expression.py --port /dev/ttyUSB0

# 循环播放
python3 src/face_robot_driver/scripts/test_expression.py --port /dev/ttyUSB0 --loop

# 单独展示某表情（保持直到 Ctrl+C）
python3 src/face_robot_driver/scripts/test_expression.py --port /dev/ttyUSB0 --hold happy
python3 src/face_robot_driver/scripts/test_expression.py --port /dev/ttyUSB0 --hold angry
python3 src/face_robot_driver/scripts/test_expression.py --port /dev/ttyUSB0 --hold sad

# 调整每个表情保持时间
python3 src/face_robot_driver/scripts/test_expression.py --port /dev/ttyUSB0 --duration 3
```

**表情涉及的舵机区域：**

| 区域 | 舵机 ID | 说明 |
|------|---------|------|
| 眉毛 | 3, 4, 7, 8 | 左右眉各2路（内/外侧），控制眉形 |
| 鼻/颊 | 1, 2, 5, 6 | 鼻翼皱起、脸颊提升 |
| 嘴角 | 18, 19, 20, 21 | 左右嘴角各2路（上下/前后） |
| 上唇 | 15, 16, 17 | 左中右三段 |
| 下唇 | 28, 29, 30 | 左中右三段 |

---

## 启动 ROS 2 节点

### 情绪镜像模式（当前主要模式）

识别人脸情绪，机器人跟随做出相同表情。需要 **两个终端**，均需先 source：

```bash
source /opt/ros/humble/setup.bash && source ~/Mark/face_robot_ws/install/setup.bash
```

**终端 1 — 驱动节点**
```bash
ros2 run face_robot_driver driver_node
```

**终端 2 — 情绪镜像节点**
```bash
# 启动（默认开启预览窗口，显示情绪分数条）
ros2 run face_robot_driver emotion_mirror

# 调整滤波参数（更稳定 / 更灵敏）
ros2 run face_robot_driver emotion_mirror --ros-args \
  -p ema_alpha:=0.2 -p min_hold_s:=0.8

# 监控识别到的情绪
ros2 topic echo /face/detected_emotion
```

### 眼球追踪模式

摄像头检测人脸位置，眼球跟随注视。与情绪镜像共用摄像头，不可同时运行。

**终端 2 — 追踪节点**
```bash
ros2 run face_robot_driver face_tracker --ros-args -p show_preview:=true
```

### 手动调试模式（eye_node）

适用于不接摄像头时测试眼部动作：

**终端 2 — 眼部节点**
```bash
ros2 run face_robot_driver eye_node
# 关闭自动闲置
ros2 run face_robot_driver eye_node --ros-args -p idle_mode:=false
```

**眼部调试命令**
```bash
# 凝视方向（pitch: -1下~+1上，yaw: -1右~+1左）
ros2 topic pub --once /face/eye/gaze_cmd std_msgs/Float32MultiArray "data: [-0.5, 0.3]"

# 眨眼 / wink / 回中
ros2 service call /face/eye/blink  std_srvs/srv/Trigger
ros2 service call /face/eye/wink   std_srvs/srv/Trigger
ros2 service call /face/eye/center std_srvs/srv/Trigger

# 开关自动闲置
ros2 service call /face/eye/idle_on  std_srvs/srv/Trigger
ros2 service call /face/eye/idle_off std_srvs/srv/Trigger
```

### 急停与恢复

```bash
ros2 service call /face/estop  std_srvs/srv/Trigger   # 急停
ros2 service call /face/resume std_srvs/srv/Trigger   # 恢复
```

---

## 配置文件说明

### `config/servos.yaml`

每路舵机的物理参数，32路已完成限位标定。

```yaml
servos:
  - id: 9
    joint: right_eyelid_upper
    neutral: 90.0        # 中位角度(°)
    min: 60.0            # 物理最小（已标定）
    max: 120.0           # 物理最大（已标定）
    direction: 1         # 1=正向, -1=反向（正向语义：张开/向上/向左）
    note: "angle↑=open"
```

| 字段 | 说明 |
|------|------|
| `direction` | 正向语义：上/张开/左为正。`raw = neutral + direction × delta` |
| `locked_by` | 依赖锁定：列表中舵机同时到达上限时本舵机强制回 neutral（id16 物理限制） |

---

## 硬件协议速查

**串口参数：** 460800 baud，8N1，无流控，fire-and-forget（无需等待回包）

### HEX 指令格式（65字节）

```
Byte  0~63 : 32路舵机占空比，每路2字节大端 uint16，范围 0~1000
Byte  64   : 和校验 = sum(Byte0~63) & 0xFF
```

### 角度 → 占空比换算

```
duty = round(angle × 5 / 9 + 25)
```

| 角度 | 占空比值 | 实际占空比 |
|------|----------|------------|
| 0°   | 25       | 2.5%       |
| 90°  | 75       | 7.5%       |
| 180° | 125      | 12.5%      |

---

## 里程碑进度

### M1 — 基础控制（已完成）

- [x] 协议层 `pom_sc32.py`：HEX 批量指令，fire-and-forget
- [x] 软限位模块 `servo_config.py`：clamp / locked_by / neutral 管理
- [x] ROS 2 驱动节点 `driver_node`：50Hz 控制循环、watchdog、急停
- [x] 32路关节命名与物理限位标定（servos.yaml）

### M2 — 眼部控制 + 追踪（已完成）

- [x] 眼部控制器 `eye_controller.py`：凝视、眼睑耦合、眨眼、自动闲置
- [x] 人脸追踪节点 `face_tracker_node`：Haar 级联 + 比例控制器

### M3 — 表情 + 情绪镜像（当前阶段）

- [x] 表情预设 `expressions.py`：happy/angry/sad，smoothstep 过渡
- [x] 表情测试脚本 `test_expression.py`
- [x] 情绪检测器 `emotion_detector.py`：FaceLandmarker blendshapes + 分类 + EMA 滤波
- [x] 情绪镜像节点 `emotion_mirror_node.py`：识别情绪 → 机器人跟随表情
- [ ] 情绪分类权重调优（根据实测调整 blendshape 权重）
- [ ] 表情角度参数调优（根据物理测试微调各舵机值）

### M4 — 计划中

- [ ] 情绪镜像 + 眼球追踪联动（合并为单节点）
- [ ] 速度限制（每周期最大角度增量，防抖动）
- [ ] Launch 文件（一键启动）

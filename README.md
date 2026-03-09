# Face Robot — 人脸表情机器人软件栈

> 基于 Ubuntu 22.04 + ROS 2 Humble，驱动 POM SC32 32路舵机控制器，实现仿人脸表情运动控制。

---

## 目录

- [项目架构](#项目架构)
- [软件层次图](#软件层次图)
- [ROS 2 接口总览](#ros-2-接口总览)
- [环境准备](#环境准备)
- [编译](#编译)
- [第一步：可行性测试（不依赖 ROS）](#第一步可行性测试不依赖-ros)
- [第二步：启动 ROS 2 节点](#第二步启动-ros-2-节点)
- [表情控制](#表情控制)
- [配置文件说明](#配置文件说明)
- [硬件协议速查](#硬件协议速查)
- [里程碑进度](#里程碑进度)

---

## 项目架构

```
face_robot_ws/
├── config/
│   ├── expressions.yaml        # 表情库：各表情对应的舵机角度
│   └── servos.yaml             # 舵机参数：限位 / 方向 / 中位
├── src/
│   └── face_robot_driver/      # 核心 ROS 2 包（ament_python）
│       ├── face_robot_driver/
│       │   ├── pom_sc32.py         # 协议层：串口通信 & HEX/REG 指令封装
│       │   ├── driver_node.py      # ROS 2 驱动节点：接收角度指令 → 串口下发
│       │   ├── expression_player.py # 表情播放器：插值过渡，线程安全
│       │   └── expression_node.py  # ROS 2 表情节点：订阅表情名 → 生成角度序列
│       ├── scripts/
│       │   └── test_servo.py       # 独立测试脚本（不依赖 ROS）
│       ├── package.xml
│       └── setup.py
└── README.md
```

---

## 软件层次图

```
┌─────────────────────────────────┐
│         用户 / 上层逻辑          │
│  ros2 topic pub / 自定义节点     │
└──────────────┬──────────────────┘
               │  /face/expression_cmd  (std_msgs/String)
               ▼
┌─────────────────────────────────┐
│        expression_node          │  加载 expressions.yaml
│  ExpressionPlayer（插值播放器）  │  线性 / 平滑步进插值
└──────────────┬──────────────────┘
               │  /face/servo_angles  (std_msgs/Float32MultiArray, 32×float)
               ▼
┌─────────────────────────────────┐
│          driver_node            │  50 Hz 控制循环
│  watchdog / 软限位 / 急停        │  watchdog 超时自动回中
└──────────────┬──────────────────┘
               │  HEX 指令（65字节，USB-Serial）
               ▼
┌─────────────────────────────────┐
│      POM SC32 驱动板             │  460800 8N1
│      32路 PWM 舵机输出           │
└─────────────────────────────────┘
```

---

## ROS 2 接口总览

### 话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/face/servo_angles` | `std_msgs/Float32MultiArray` | 输入→driver_node | 32个角度值(0-180°)，索引0对应舵机1 |
| `/face/expression_cmd` | `std_msgs/String` | 输入→expression_node | 表情名称，可附时长，格式：`smile` 或 `smile,400` |

### 服务

| 服务 | 类型 | 节点 | 说明 |
|------|------|------|------|
| `/face/estop` | `std_srvs/Trigger` | driver_node | 急停：立即回中，拒绝后续指令 |
| `/face/resume` | `std_srvs/Trigger` | driver_node | 解除急停 |

### 节点参数

**driver_node**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `port` | `/dev/ttyUSB0` | 串口设备路径 |
| `baudrate` | `460800` | 波特率 |
| `pwm_freq` | `50` | PWM 频率(Hz)，标准舵机用50 |
| `max_rate` | `50.0` | 控制循环频率上限(Hz) |
| `watchdog_timeout` | `1.0` | 掉线超时时间(s)，超时后自动回中 |

**expression_node**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `expressions_yaml` | `~/face_robot_ws/config/expressions.yaml` | 表情配置文件路径 |

---

## 环境准备

### 1. 系统依赖

```bash
# ROS 2 Humble（已安装则跳过）
sudo apt install -y ros-humble-desktop

# Python 串口库
pip3 install pyserial pyyaml

# 串口访问权限（需重新登录生效）
sudo usermod -a -G dialout $USER
```

### 2. 验证串口识别

```bash
# 连接驱动板 USB 后
ls /dev/ttyUSB*
# 应出现 /dev/ttyUSB0 或类似设备
```

---

## 编译

```bash
cd /home/tuf/Mark/face_robot_ws

# 首次编译（或新增包后）
source /opt/ros/humble/setup.bash
colcon build

# 开发时用 symlink 模式（修改 Python 文件无需重新编译）
colcon build --symlink-install

# 加载工作空间环境（每个新终端都需要执行）
source /home/tuf/Mark/face_robot_ws/install/setup.bash
```

---

## 第一步：可行性测试（不依赖 ROS）

**连接硬件后先用此脚本验证串口通信是否正常，无需启动 ROS。**

```bash
cd /home/tuf/Mark/face_robot_ws

# 单舵机正弦扫描（舵机1，0-180° 往复，验证通信链路）
python3 src/face_robot_driver/scripts/test_servo.py --port /dev/ttyUSB0

# 指定舵机到指定角度
python3 src/face_robot_driver/scripts/test_servo.py --port /dev/ttyUSB0 --servo 1 --angle 90

# 32路全部回中
python3 src/face_robot_driver/scripts/test_servo.py --port /dev/ttyUSB0 --all
```

**预期输出（正常）：**
```
连接串口: /dev/ttyUSB0 @ 460800
初始化驱动板（设置PWM频率50Hz）...
=== 单舵机测试: 舵机#1 ===
  角度=  0.0°  duty=25  OK=True
  角度= 11.2°  duty=31  OK=True
  ...
```

若 `OK=False`：驱动板已动作但未收到确认，通信基本正常；若舵机完全无反应，检查接线与波特率。

---

## 第二步：启动 ROS 2 节点

需要打开 **两个终端**，分别运行驱动节点和表情节点。

### 终端 1 — 驱动节点

```bash
source /opt/ros/humble/setup.bash
source /home/tuf/Mark/face_robot_ws/install/setup.bash

ros2 run face_robot_driver driver_node
```

**自定义串口：**
```bash
ros2 run face_robot_driver driver_node --ros-args -p port:=/dev/ttyUSB0 -p watchdog_timeout:=2.0
```

### 终端 2 — 眼部节点

--                                                                           
  架构设计
                                                                                
  EyeMech 参考点 → 我们的实现映射：                                           
                                                                                
  EyeMech: LR/UD eyeball + TL/TR/BL/BR eyelids                                  
  本机:    id14(yaw)/id13(pitch) + id9/10/11/12 eyelids                         
                                                                                
  eye_controller.py 核心逻辑                                                    
                                                                                
  ┌───────────────────────────┬──────────────────────────────────────┐          
  │           功能             │                 实现                 │        
  ├───────────────────────────┼──────────────────────────────────────┤          
  │ 凝视 set_gaze(pitch, yaw) │ smoothstep 插值，~55Hz               │        
  ├───────────────────────────┼──────────────────────────────────────┤
  │ 眼睑耦合                  │ 向下看时上眼睑跟随35%，下眼睑跟随18% │          
  ├───────────────────────────┼──────────────────────────────────────┤          
  │ 眨眼 blink()              │ 关闭80ms → 停留35ms → 张开130ms      │          
  ├───────────────────────────┼──────────────────────────────────────┤          
  │ 半眨眼 blink(half=True)   │ 闭合到50%，困倦感                    │        
  ├───────────────────────────┼──────────────────────────────────────┤          
  │ Wink wink('right'/'left') │ 单眼独立                             │        
  ├───────────────────────────┼──────────────────────────────────────┤          
  │ 自动闲置 start_idle()     │ 60%微扫视 + 40%大扫视，3-7s眨眼      │        
  └───────────────────────────┴──────────────────────────────────────┘          
                                                                              
  ---                                                                           
  启动调试步骤                                                                
                                                                                
  # 终端1：驱动节点
  ros2 run face_robot_driver driver_node                                        
                                                                              
  # 终端2：眼部节点（先关闭idle，方便手动测试）                                 
  ros2 run face_robot_driver eye_node --ros-args -p idle_mode:=false          
                                                                                
  # 测试眨眼                                                                  
  ros2 service call /face/eye/blink std_srvs/srv/Trigger                        
                                                                                
  # 测试向右下看
  ros2 topic pub --once /face/eye/gaze_cmd std_msgs/Float32MultiArray "data:    
  [-0.6, -0.5]"                                                                 
   
  # 确认参数合适后开启自动闲置                                                  
  ros2 service call /face/eye/idle_on std_srvs/srv/Trigger                    
                                                                                
  可调参数（在 eye_controller.py 顶部）：                                       
  - _UPPER_COUPLING = 0.35 — 上眼睑跟随俯仰的程度
  - _LOWER_COUPLING = 0.18 — 下眼睑耦合                                         
  - blink() 中的 80ms/35ms/130ms 时序        

```bash
source /opt/ros/humble/setup.bash
source /home/tuf/Mark/face_robot_ws/install/setup.bash

# 默认启动（自动闲置开启）
ros2 run face_robot_driver eye_node \
  --ros-args -p servos_yaml:=/home/tuf/Mark/face_robot_ws/config/servos.yaml

# 关闭自动闲置（手动调试模式）
ros2 run face_robot_driver eye_node --ros-args -p idle_mode:=false
```

**眼部调试命令：**

```bash
# 触发眨眼
ros2 service call /face/eye/blink std_srvs/srv/Trigger

# 触发右眼 wink
ros2 service call /face/eye/wink std_srvs/srv/Trigger

# 眼球回正中
ros2 service call /face/eye/center std_srvs/srv/Trigger

# 设置凝视方向 [pitch, yaw]  pitch:-1(下)~+1(上)  yaw:-1(右)~+1(左)
ros2 topic pub --once /face/eye/gaze_cmd std_msgs/Float32MultiArray "data: [-0.5, 0.3]"

# 设置凝视 + 自定义时长 [pitch, yaw, duration_ms]
ros2 topic pub --once /face/eye/gaze_cmd std_msgs/Float32MultiArray "data: [0.0, -0.8, 200.0]"

# 开关自动闲置
ros2 service call /face/eye/idle_on  std_srvs/srv/Trigger
ros2 service call /face/eye/idle_off std_srvs/srv/Trigger
```

### 终端 3（可选）— 表情节点

```bash
source /opt/ros/humble/setup.bash
source /home/tuf/Mark/face_robot_ws/install/setup.bash

ros2 run face_robot_driver expression_node \
  --ros-args -p expressions_yaml:=/home/tuf/Mark/face_robot_ws/config/expressions.yaml
```

---

## 表情控制

### 方式一：命令行快速发布（测试用）

```bash
# 播放 smile 表情（默认500ms过渡）
ros2 topic pub --once /face/expression_cmd std_msgs/String "data: 'smile'"

# 播放 surprise，过渡时长300ms
ros2 topic pub --once /face/expression_cmd std_msgs/String "data: 'surprise,300'"

# 回到中性表情
ros2 topic pub --once /face/expression_cmd std_msgs/String "data: 'neutral'"
```

### 方式二：直接发布角度（精确控制）

```bash
# 发布32路角度（全部90°）
ros2 topic pub --once /face/servo_angles std_msgs/Float32MultiArray \
  "data: [90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90]"
```

### 急停与恢复

```bash
# 急停（立即回中，拒绝新指令）
ros2 service call /face/estop std_srvs/srv/Trigger

# 解除急停
ros2 service call /face/resume std_srvs/srv/Trigger
```

### 查看当前话题数据

```bash
# 监控角度指令
ros2 topic echo /face/servo_angles

# 查看节点状态
ros2 node list
ros2 node info /servo_driver
```

---

## 配置文件说明

### `config/expressions.yaml`

定义各表情的舵机角度。`servos` 字段中只需填写需要变化的舵机，未填写的舵机保持当前位置。

```yaml
expressions:

  neutral:
    duration_ms: 500
    servos: {}          # 所有舵机的中位，根据实际标定后填入

  smile:
    duration_ms: 400
    servos:
      1: 110            # 舵机1（左嘴角）→ 110°
      2: 70             # 舵机2（右嘴角）→ 70°
      # ... 更多关节
```

**字段说明：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `duration_ms` | int | 表情过渡时长（毫秒），可被调用时覆盖 |
| `servos` | dict | `舵机编号(1-32): 目标角度(0-180°)` |

### `config/servos.yaml`

每路舵机的物理参数。32路关节已完成限位标定，direction 待统一校验。

```yaml
servos:
  - id: 1
    joint: right_nose          # 面部关节名称
    neutral: 90.0              # 中位角度(°)
    min: 80.0                  # 物理最小角度（已标定）
    max: 160.0                 # 物理最大角度（已标定）
    direction: 1               # 1=正向, -1=反向（待校验）
    note: "可选备注"
    locked_by: [15, 17]        # 可选：当依赖舵机到达上限时，本舵机锁定回中位
```

**特殊字段说明：**

| 字段 | 说明 |
|------|------|
| `locked_by` | 依赖锁定：列表中的舵机同时到达上限时，本舵机强制回 neutral（用于 id16 上唇中部的物理限制） |
| `note` | 运动方向备注，供标定参考 |

**已锁定（min=max=neutral）的关节：** id22-27（下颌/舌头）、id31-32（脖子），暂不参与表情控制。

---

## 硬件协议速查

**串口参数：** 460800 baud，8N1，无校验，无流控

### HEX 指令格式（65字节）

```
[Byte 0-63]  32路舵机占空比，每路2字节大端 uint16，范围 0-1000
[Byte 64]    和校验 = sum(Byte0~63) & 0xFF
```

响应：`OK_HEX`（6字节）

### 角度→占空比换算

```
duty = round(angle × 5 / 9 + 25)
```

| 角度 | 占空比值 | 实际占空比 |
|------|----------|------------|
| 0°   | 25       | 2.5%       |
| 90°  | 75       | 7.5%       |
| 180° | 125      | 12.5%      |

### PWM 分组（初始化时全部设为50Hz）

| 组 | 包含舵机通道 |
|----|-------------|
| 组1 | 1, 4, 5, 6, 7, 8 |
| 组2 | 17 |
| 组3 | 11, 12, 13, 14, 15, 16, 31, 32 |
| 组4 | 19, 20, 21, 24, 25, 26 |
| 组5 | 9, 10, 18, 27, 28, 29, 30 |
| 组6 | 2, 3, 22, 23 |

---

## 里程碑进度

### M1 — 基础控制（当前阶段）

- [x] 协议分析：从例程提取 HEX/REG 帧格式、校验、波特率
- [x] 串口通信层 `pom_sc32.py`：HEX 批量指令、REG 单路指令
- [x] 可行性测试脚本 `test_servo.py`（不依赖 ROS）
- [x] ROS 2 驱动节点 `driver_node`：50Hz 控制循环、watchdog、急停服务
- [x] 表情播放器 `expression_player`：线性/平滑步进插值、可中断
- [x] ROS 2 表情节点 `expression_node`：订阅表情名 → 发布角度序列
- [x] 配置文件 `expressions.yaml` / `servos.yaml` 骨架
- [x] 32路舵机关节分配与命名完成
- [x] 32路关节物理限位标定完成（min/max/neutral 已写入 `servos.yaml`）
- [x] 软限位模块 `servo_config.py`：clamp、locked_by 约束、neutral 管理
- [x] `driver_node` 集成软限位，启动时以真实 neutral 回中
- [x] 方向参数 `direction` 统一校验完成（已建立语义约定并修正 id4/8/18/20/29）
- [x] 眼部控制器 `eye_controller.py`：凝视、眼睑耦合、眨眼、自动闲置
- [x] ROS 2 眼部节点 `eye_node`：凝视话题、眨眼/wink/回中服务、idle开关
- [ ] 眼部参数调优（耦合系数、眨眼时序）
- [ ] 表情库角度填入 `expressions.yaml`（neutral / smile / frown / surprise / blink）

### M2 — 计划中

- [ ] 速度限制（每周期最大角度增量，防抖动）
- [ ] Launch 文件（一键启动所有节点）
- [ ] 更多表情预设

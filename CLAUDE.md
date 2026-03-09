# Face Robot (Morpheus-like) — claude.md

## 项目概述
[project]
- 目标：在 Ubuntu 22.04 + ROS 2 Humble上开发一套“人脸表情机器人”软件栈
- 硬件参考：Morpheus（开源硬件与软件仓库，可作为结构/理念参考，不强依赖其算法实现）https://github.com/ZZongzheng0918/Morpheus-Software
- 当前阶段（里程碑 M1）：能通过 ROS 2 话题/服务/动作指令，让若干舵机按“表情配置”可靠运动（含限幅、速率限制、急停与掉线保护）。

## 技术栈
[stack]
- OS / Middleware
  - Ubuntu 22.04, ROS 2 Humble（colcon + ament_cmake / ament_python）
- 语言
  - Python: rclpy, pyserial（快速打通通信与调试）
  - C++: rclcpp, Boost.Asio 或第三方 serial 库（更低延迟、更稳定）
- 通信与协议
  - USB-Serial (CH340/CP2102 等) + 驱动板私有串口协议（需从例程提取帧格式/校验/波特率）
- 控制框架
  - 轻量自研：自定义 topic/service/action + 表情播放器（最快落地）

## 开发指令
[commands]
### 1) 基础环境

- 常用工具
  - sudo apt install -y python3-venv python3-pip minicom screen picocom
  - sudo usermod -a -G dialout $USER  # 允许访问 /dev/ttyUSB*
  - 重新登录生效

### 2) 工作空间结构
- mkdir -p ~/face_robot_ws/src
- cd ~/face_robot_ws/src
- （后续放入多个 packages：msgs/driver/expression/description/bringup）
- cd ~/face_robot_ws
- colcon build
- colcon build --symlink-install
- source install/setup.bash

## 当前任务
[tasks]
1. 协议与链路打通

       从驱动板例程抽取：波特率、帧格式、校验/CRC、单舵机/多舵机命令、回包格式，在 Ubuntu 上用 CLI（Python 或 C++）稳定控制 1 个舵机，例程位于：POM_SC32文件夹中

2. 多舵机批量控制能力

      支持一次下发 N 路舵机目标

     固定更新频率（例如 50 Hz 上限），不会刷爆串口或导致抖动

3. 标定与参数化

   servos.yaml：每路舵机的 min/max/neutral/offset/direction 等参数

      角度/脉宽单位统一（内部统一用“角度/弧度”，协议层再换算）

4. ROS2 驱动节点

      提供 ROS2 接口：

      低层：topic（例如 /face/servo_cmd）用于实时控制

      高层：service/action（例如 /face/play_expression）用于播放表情

5. 表情库与播放器

   expressions.yaml：至少 3~5 个表情预设（neutral/smile/frown/surprise/blink…）

      播放器支持：平滑插值（线性/三次）、duration、可中断/切换

6. 安全与稳定性机制（必须）

      软限位（clamp）+ 速度限制（每周期最大增量）

      掉线/超时回中（watchdog）

      急停（E-stop）与安全位（safe pose）

# Face Robot (Morpheus-like) — claude.md

## 项目概述
[project]
- 目标：在 Ubuntu 22.04 + ROS 2 Humble上开发一套“人脸表情机器人”软件栈
- 硬件参考：Morpheus（可作为结构/理念参考，不强依赖其算法实现）https://github.com/ZZongzheng0918/Morpheus-Software，眼部有一个uvc摄像头
- 阶段一：通过 ROS 2 话题/服务/动作指令，实现舵机的控制，含限幅、速率限制、急停与掉线保护（已完成）
- 阶段二：搭建眼部动作节点，实现眼部动作的控制，参考控制代码，实现眼睛对人脸的追踪（已完成）
- 阶段三：实现嘴部以及眉毛部分的动作，可以跟踪输入图像中人物的表情，（已完成）
- 阶段四：实现眼部、嘴部、眉毛部分动作的协调运行，接入大模型实现语音对话以及环境感知，输入语音实现对应的嘴部动作（未完成）

## 技术栈
[stack]
- OS / Middleware
  - Ubuntu 22.04, ROS 2 Humble（colcon + ament_cmake / ament_python）
- 语言
  - Python: rclpy, pyserial
  - C++: rclcpp, Boost.Asio 或第三方 serial 库
- 通信与协议
  - USB-Serial (CH340/CP2102 等) + 驱动板私有串口协议（需从例程提取帧格式/校验/波特率）
- 控制框架
  - 轻量自研：自定义 topic/service/action + 表情播放器

## 开发指令
[commands]
### 1) 基础环境

- 常用工具
  - sudo apt install -y python3-venv python3-pip minicom screen picocom
  - sudo usermod -a -G dialout $USER  # 允许访问 /dev/ttyUSB*
  - 重新登录生效
  - 如非必须情况，禁止随意使用pip3进行功能包安装，严肃进行包管理，防止系统崩溃，尽可能使用apt进行安装
  - 在每次更新代码后及时更新readme文件

### 2) 工作空间结构
- mkdir -p ~/face_robot_ws/src
- cd ~/face_robot_ws/src
- （后续放入多个 packages）
- cd ~/face_robot_ws
- colcon build
- colcon build --symlink-install
- source install/setup.bash

### 3) 账号信息
- APP ID: 6496508405
- Access Token: iU76WCb5fJU1w_mmdiTnyAZOqlJVJLpF
- Secret Key: P2-u8affmJsdKwFLP_yWh9KgXV34HxXM

## 当前任务
[tasks]
1. 由对话中给出具体任务

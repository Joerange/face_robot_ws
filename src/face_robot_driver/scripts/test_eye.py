#!/usr/bin/env python3
"""
眼部动作独立测试脚本 (不依赖 ROS)

用法:
  python3 test_eye.py --port /dev/ttyUSB0 --blink          # 单次眨眼
  python3 test_eye.py --port /dev/ttyUSB0 --wink right     # 右眼 wink
  python3 test_eye.py --port /dev/ttyUSB0 --gaze 0 0.5     # 眼球右转 50%
  python3 test_eye.py --port /dev/ttyUSB0 --idle           # 自动闲置模式（Ctrl+C 退出）
  python3 test_eye.py --port /dev/ttyUSB0 --open           # 强制张眼
  python3 test_eye.py --port /dev/ttyUSB0 --close          # 强制闭眼
"""

import sys
import os
import argparse
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from face_robot_driver.pom_sc32 import PomSC32
from face_robot_driver.servo_config import ServoConfig
from face_robot_driver.eye_controller import EyeController

import logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

SERVOS_YAML = os.path.join(os.path.dirname(__file__),
                           '../../../config/servos.yaml')


def make_eye(driver: PomSC32, cfg: ServoConfig) -> EyeController:
    def send(angles):
        driver.set_angles(angles)
    eye = EyeController(cfg, on_update=send)
    return eye


def main():
    parser = argparse.ArgumentParser(description='眼部舵机测试')
    parser.add_argument('--port',  default='/dev/ttyUSB0')
    parser.add_argument('--baud',  type=int, default=460800)
    parser.add_argument('--yaml',  default=SERVOS_YAML)

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--blink',  action='store_true',        help='单次眨眼')
    group.add_argument('--wink',   choices=['right', 'left'],  help='单眼 wink')
    group.add_argument('--gaze',   nargs=2, type=float,        metavar=('PITCH', 'YAW'),
                       help='凝视目标 pitch[-1~1] yaw[-1~1]')
    group.add_argument('--idle',   action='store_true',        help='自动闲置模式')
    group.add_argument('--open',   action='store_true',        help='张开眼睛')
    group.add_argument('--close',  action='store_true',        help='闭合眼睛')
    group.add_argument('--center', action='store_true',        help='眼球回正中')

    args = parser.parse_args()

    cfg = ServoConfig(os.path.realpath(args.yaml))

    import serial

    print(f"连接串口: {args.port} @ {args.baud}")
    try:
        driver_ctx = PomSC32(args.port, args.baud)
    except serial.SerialException as e:
        print(f"串口错误: {e}")
        print("请检查: 1) 设备已连接  2) 权限(newgrp dialout)")
        sys.exit(1)

    with driver_ctx as driver:
        driver.initialize(freq=50)
        time.sleep(0.1)

        eye = make_eye(driver, cfg)
        print("眼部控制器已初始化（眼睛张开，居中）")
        time.sleep(0.3)

        if args.blink:
            print("执行眨眼...")
            eye.blink()
            print("完成")
            time.sleep(0.5)

        elif args.wink:
            print(f"执行 {args.wink} 眼 wink...")
            eye.wink(side=args.wink)
            print("完成")
            time.sleep(0.5)

        elif args.gaze:
            pitch, yaw = args.gaze
            print(f"凝视: pitch={pitch:.2f}, yaw={yaw:.2f}")
            eye.set_gaze(pitch, yaw, duration_ms=200)
            print(f"已到达，保持位置，按 Ctrl+C 退出")
            try:
                while True:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\n回正中...")
                eye.center_gaze()

        elif args.idle:
            print("自动闲置模式（随机眨眼+扫视），按 Ctrl+C 退出")
            eye.start_idle()
            try:
                while True:
                    s = eye.state
                    print(f"\r  pitch={s['pitch']:+.2f}  yaw={s['yaw']:+.2f}"
                          f"  lid={s['lid_open']:.2f}", end='', flush=True)
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\n停止闲置...")
                eye.stop_idle()
                eye.center_gaze()

        elif args.open:
            print("张开眼睛...")
            eye.open_eyes()
            print("完成，保持，按 Ctrl+C 退出")
            try:
                while True:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                pass

        elif args.close:
            print("闭合眼睛...")
            eye.close_eyes()
            print("完成，保持，按 Ctrl+C 退出")
            try:
                while True:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\n张开眼睛...")
                eye.open_eyes()

        elif args.center:
            print("眼球回正中...")
            eye.center_gaze()
            print("完成")
            time.sleep(0.5)

        print("测试结束")


if __name__ == '__main__':
    main()

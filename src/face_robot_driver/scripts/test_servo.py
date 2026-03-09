#!/usr/bin/env python3
"""
可行性测试脚本 (不依赖 ROS)
用法:
  python3 test_servo.py --port /dev/ttyUSB0                        # 单舵机正弦扫描
  python3 test_servo.py --port /dev/ttyUSB0 --all                  # 32路全部回 neutral
  python3 test_servo.py --port /dev/ttyUSB0 --servo 1 --angle 80   # 设置舵机1到80°，保持不动
"""

import sys
import os
import argparse
import time
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from face_robot_driver.pom_sc32 import PomSC32, angle_to_duty
from face_robot_driver.servo_config import ServoConfig

import logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

SERVOS_YAML = os.path.join(os.path.dirname(__file__),
                           '../../../config/servos.yaml')


def go_neutral(driver: PomSC32, cfg: ServoConfig):
    """所有舵机回到各自的 neutral 角度（读取 servos.yaml）。"""
    neutrals = cfg.neutral_angles()
    ok = driver.set_angles(neutrals)
    return ok


def test_single_servo(driver: PomSC32, cfg: ServoConfig, servo_id: int):
    """对单个舵机在其 min-max 范围内做正弦扫描。"""
    p = cfg.get(servo_id)
    neutrals = cfg.neutral_angles()
    print(f"\n=== 正弦扫描: 舵机#{servo_id} ({p.joint}) ===")
    print(f"  范围: [{p.min}°, {p.max}°]  neutral={p.neutral}°")

    angles = list(neutrals)
    steps = 40
    for i in range(steps + 1):
        t = i / steps * 2 * math.pi
        # 在 min-max 范围内正弦扫描
        angle = p.min + (p.max - p.min) * (math.sin(t) + 1) / 2
        angles[servo_id - 1] = angle
        driver.set_angles(angles)
        print(f"  角度={angle:6.1f}°  duty={angle_to_duty(angle)}")
        time.sleep(0.05)

    print(f"  扫描完成，保持末位（不回中）")


def test_all_neutral(driver: PomSC32, cfg: ServoConfig):
    """32路全部回到各自 neutral。"""
    neutrals = cfg.neutral_angles()
    print("\n=== 32路回 neutral ===")
    for i, n in enumerate(neutrals):
        p = cfg.get(i + 1)
        print(f"  id{i+1:2d} {p.joint:<28} → {n}°")
    driver.set_angles(neutrals)
    print(f"\n  已发送 neutral 指令")


def test_set_angle(driver: PomSC32, cfg: ServoConfig, servo_id: int, angle: float):
    """设置指定舵机到指定角度，保持不动。"""
    p = cfg.get(servo_id)
    neutrals = cfg.neutral_angles()
    print(f"\n=== 设置舵机#{servo_id} ({p.joint}) → {angle}° ===")
    print(f"  范围: [{p.min}°, {p.max}°]  neutral={p.neutral}°")

    # 软限位提示
    clamped = max(p.min, min(p.max, angle))
    if abs(clamped - angle) > 0.1:
        print(f"  ⚠ 角度超出限位，已 clamp 至 {clamped}°")

    angles = list(neutrals)
    angles[servo_id - 1] = clamped
    ok = driver.set_angles(angles)
    print(f"  duty={angle_to_duty(clamped)}  结果: {'成功' if ok else '未收到确认'}")
    print(f"  舵机保持在 {clamped}°，按 Ctrl+C 退出")

    # 保持角度，等待用户中断
    try:
        while True:
            driver.set_angles(angles)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n  用户中断")


def main():
    parser = argparse.ArgumentParser(description='POM SC32 舵机测试')
    parser.add_argument('--port',  default='/dev/ttyUSB0')
    parser.add_argument('--baud',  type=int, default=460800)
    parser.add_argument('--servo', type=int, default=1, help='舵机编号(1-32)')
    parser.add_argument('--angle', type=float, default=None, help='目标角度(0-180)')
    parser.add_argument('--all',   action='store_true', help='32路全部回 neutral')
    parser.add_argument('--yaml',  default=SERVOS_YAML, help='servos.yaml 路径')
    args = parser.parse_args()

    cfg = ServoConfig(os.path.realpath(args.yaml))

    print(f"连接串口: {args.port} @ {args.baud}")
    try:
        with PomSC32(args.port, args.baud) as driver:
            print("初始化驱动板（设置PWM频率50Hz）...")
            driver.initialize(freq=50)
            time.sleep(0.1)

            if args.all:
                test_all_neutral(driver, cfg)
            elif args.angle is not None:
                test_set_angle(driver, cfg, args.servo, args.angle)
            else:
                test_single_servo(driver, cfg, args.servo)

    except Exception as e:
        import serial
        if isinstance(e, serial.SerialException):
            print(f"串口错误: {e}")
            print("请检查: 1) 设备已连接  2) 权限(newgrp dialout)")
        else:
            raise
        sys.exit(1)


if __name__ == '__main__':
    main()

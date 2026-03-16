#!/usr/bin/env python3
"""
表情测试脚本（不依赖 ROS）

依次展示 4 种表情，每种保持 5 秒，平滑过渡：
  开心 → 正常 → 生气 → 伤心 → 正常

用法:
  python3 test_expression.py --port /dev/ttyUSB0
  python3 test_expression.py --port /dev/ttyUSB0 --hold happy   # 单独保持某表情
  python3 test_expression.py --port /dev/ttyUSB0 --loop          # 循环播放
"""

import sys
import os
import argparse
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from face_robot_driver.pom_sc32 import PomSC32
from face_robot_driver.servo_config import ServoConfig
from face_robot_driver.expressions import EXPRESSIONS, smoothstep

SERVOS_YAML = os.path.join(os.path.dirname(__file__),
                           '../../../config/servos.yaml')


def transition(driver, cfg, current, expr_overrides, duration=0.6):
    """
    从 current(32路角度列表) 平滑过渡到目标表情。
    expr_overrides: {servo_id: angle} 非 neutral 的覆盖值。
    未在 overrides 中的舵机回到 servos.yaml 的 neutral。
    返回过渡后的角度列表。
    """
    neutral = cfg.neutral_angles()
    target = list(neutral)
    for sid, angle in expr_overrides.items():
        target[sid - 1] = angle

    # clamp
    target = cfg.apply(target)

    dt = 0.02   # 50Hz
    steps = max(1, int(duration / dt))

    for i in range(steps + 1):
        t = smoothstep(i / steps)
        angles = [c + (g - c) * t for c, g in zip(current, target)]
        driver.set_angles(angles)
        if i < steps:
            time.sleep(dt)

    return target


def main():
    parser = argparse.ArgumentParser(description='表情测试（眉毛+嘴部）')
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=460800)
    parser.add_argument('--yaml', default=SERVOS_YAML)
    parser.add_argument('--hold', choices=list(EXPRESSIONS.keys()),
                        help='单独展示某表情并保持')
    parser.add_argument('--loop', action='store_true',
                        help='循环播放全部表情')
    parser.add_argument('--duration', type=float, default=5.0,
                        help='每个表情保持时间(s)，默认5')
    args = parser.parse_args()

    import serial
    cfg = ServoConfig(os.path.realpath(args.yaml))

    try:
        driver_ctx = PomSC32(args.port, args.baud)
    except serial.SerialException as e:
        print(f"串口错误: {e}")
        sys.exit(1)

    with driver_ctx as driver:
        driver.initialize(freq=50)
        time.sleep(0.1)

        neutral = cfg.neutral_angles()
        driver.set_angles(neutral)
        current = list(neutral)
        print("初始化完成，回到 neutral")
        time.sleep(1.0)

        if args.hold:
            # 单独展示一个表情
            name = args.hold
            overrides = EXPRESSIONS[name]
            print(f"  >>> {name}")
            current = transition(driver, cfg, current, overrides)
            print(f"  保持中，按 Ctrl+C 退出")
            try:
                while True:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\n  回到 neutral...")
                transition(driver, cfg, current, {})
        else:
            # 序列播放
            sequence = [
                ('happy',   '开心'),
                ('neutral', '正常'),
                ('angry',   '生气'),
                ('neutral', '正常'),
                ('sad',     '伤心'),
                ('neutral', '正常'),
            ]

            try:
                while True:
                    for name, label in sequence:
                        overrides = EXPRESSIONS.get(name, {})
                        print(f"  >>> {label} ({name})  [{args.duration}s]")
                        current = transition(driver, cfg, current, overrides)
                        time.sleep(args.duration)

                    if not args.loop:
                        break
                    print("  --- 循环重新开始 ---")

            except KeyboardInterrupt:
                print("\n  用户中断，回到 neutral...")
                transition(driver, cfg, current, {})

        print("测试结束")


if __name__ == '__main__':
    main()

"""
ROS 2 舵机驱动节点
订阅 /face/servo_angles (Float32MultiArray, 32个角度值 0-180°)
以固定频率(max_rate Hz)通过 HEX 指令下发给 POM SC32
内置：软限位(clamp)、watchdog 超时回中、急停服务
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger

import threading
import time
import os
import serial

from .pom_sc32 import PomSC32, NUM_SERVOS
from .servo_config import ServoConfig


class ServoDriverNode(Node):

    def __init__(self):
        super().__init__('servo_driver')

        # --- 参数 ---
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 460800)
        self.declare_parameter('pwm_freq', 50)
        self.declare_parameter('max_rate', 50.0)
        self.declare_parameter('watchdog_timeout', 1.0)
        self.declare_parameter('servos_yaml',
            os.path.expanduser('~/Mark/face_robot_ws/config/servos.yaml'))

        port      = self.get_parameter('port').value
        baud      = self.get_parameter('baudrate').value
        freq      = self.get_parameter('pwm_freq').value
        rate_hz   = self.get_parameter('max_rate').value
        self._wd_timeout  = self.get_parameter('watchdog_timeout').value
        servos_yaml = self.get_parameter('servos_yaml').value

        # --- 软限位配置 ---
        self._cfg = ServoConfig(servos_yaml)
        self.get_logger().info(f'已加载舵机限位配置: {servos_yaml}')

        # --- 状态（使用实际 neutral 作为初始目标，而非硬编码90°）---
        self._target_angles = self._cfg.neutral_angles()
        self._lock = threading.Lock()
        self._last_cmd_time = time.time()
        self._estop = False

        # --- 串口驱动 ---
        try:
            self._driver = PomSC32(port, baud)
            self._driver.initialize(freq=freq)
            # 用各舵机真实 neutral 回中，而非 90°
            self._driver.set_angles(self._target_angles)
            self.get_logger().info(f'POM SC32 初始化完成: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'串口打开失败: {e}')
            raise

        # --- ROS 接口 ---
        self._sub = self.create_subscription(
            Float32MultiArray,
            '/face/servo_angles',
            self._angle_callback,
            10
        )

        self._estop_srv = self.create_service(
            Trigger, '/face/estop', self._estop_callback
        )
        self._resume_srv = self.create_service(
            Trigger, '/face/resume', self._resume_callback
        )

        # --- 控制循环 ---
        self._timer = self.create_timer(1.0 / rate_hz, self._control_loop)

        self.get_logger().info(
            f'驱动节点已启动  频率={rate_hz}Hz  watchdog={self._wd_timeout}s'
        )

    # ------------------------------------------------------------------
    def _angle_callback(self, msg: Float32MultiArray):
        if self._estop:
            return
        data = list(msg.data)
        if len(data) != NUM_SERVOS:
            self.get_logger().warn(
                f'期望32个角度值，收到{len(data)}个，忽略', throttle_duration_sec=2.0
            )
            return
        # 软限位 clamp（含 locked_by 约束）
        clamped = self._cfg.apply(data)
        with self._lock:
            self._target_angles = clamped
            self._last_cmd_time = time.time()

    def _estop_callback(self, _req, response):
        self._estop = True
        neutral = self._cfg.neutral_angles()
        self._driver.set_angles(neutral)
        self.get_logger().warn('急停触发，舵机已回中位')
        response.success = True
        response.message = 'E-Stop activated, servos at neutral'
        return response

    def _resume_callback(self, _req, response):
        self._estop = False
        with self._lock:
            self._last_cmd_time = time.time()
        self.get_logger().info('急停解除')
        response.success = True
        response.message = 'Resumed'
        return response

    # ------------------------------------------------------------------
    def _control_loop(self):
        if self._estop:
            return

        now = time.time()
        with self._lock:
            elapsed = now - self._last_cmd_time
            angles = list(self._target_angles)

        if elapsed > self._wd_timeout:
            self.get_logger().warn(
                f'Watchdog触发({elapsed:.1f}s无指令)，回中位',
                throttle_duration_sec=5.0
            )
            self._driver.set_angles(self._cfg.neutral_angles())
            return

        self._driver.set_angles(angles)

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.get_logger().info('节点退出，舵机回中...')
        try:
            self._driver.set_angles(self._cfg.neutral_angles())
            self._driver.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ServoDriverNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'节点异常: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

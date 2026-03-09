"""
ROS 2 眼部控制节点

话题（订阅）:
  /face/eye/gaze_cmd  Float32MultiArray  [pitch, yaw] 或 [pitch, yaw, duration_ms]

话题（发布）:
  /face/servo_angles  Float32MultiArray  32路角度

服务:
  /face/eye/blink     std_srvs/Trigger   触发一次眨眼
  /face/eye/wink      std_srvs/Trigger   触发右眼wink（可通过参数选侧）
  /face/eye/center    std_srvs/Trigger   眼球回正中
  /face/eye/idle_on   std_srvs/Trigger   开启自动闲置
  /face/eye/idle_off  std_srvs/Trigger   关闭自动闲置

参数:
  servos_yaml    str   servos.yaml 路径
  idle_mode      bool  启动时是否开启自动闲置（默认 True）
  publish_rate   float 发布频率 Hz（默认 50）
"""

import os
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger

from .servo_config import ServoConfig
from .eye_controller import EyeController


class EyeNode(Node):

    def __init__(self):
        super().__init__('eye_node')

        # --- 参数 ---
        self.declare_parameter(
            'servos_yaml',
            os.path.expanduser('~/Mark/face_robot_ws/config/servos.yaml')
        )
        self.declare_parameter('idle_mode', True)
        self.declare_parameter('publish_rate', 50.0)

        yaml_path = self.get_parameter('servos_yaml').value
        idle_mode = self.get_parameter('idle_mode').value
        rate_hz   = self.get_parameter('publish_rate').value

        # --- 舵机配置 & 眼部控制器 ---
        cfg = ServoConfig(yaml_path)
        self._eye = EyeController(cfg, on_update=self._on_angles_update)
        self._current_angles = cfg.neutral_angles()

        # --- 发布 ---
        self._pub = self.create_publisher(Float32MultiArray, '/face/servo_angles', 10)

        # --- 订阅凝视指令 ---
        self.create_subscription(
            Float32MultiArray,
            '/face/eye/gaze_cmd',
            self._gaze_cb,
            10
        )

        # --- 服务 ---
        self.create_service(Trigger, '/face/eye/blink',    self._blink_cb)
        self.create_service(Trigger, '/face/eye/wink',     self._wink_cb)
        self.create_service(Trigger, '/face/eye/center',   self._center_cb)
        self.create_service(Trigger, '/face/eye/idle_on',  self._idle_on_cb)
        self.create_service(Trigger, '/face/eye/idle_off', self._idle_off_cb)

        # --- 定时发布 ---
        self.create_timer(1.0 / rate_hz, self._publish_timer)

        # --- 启动 idle ---
        if idle_mode:
            self._eye.start_idle()
            self.get_logger().info('眼部节点已启动（自动闲置：开启）')
        else:
            self.get_logger().info('眼部节点已启动（自动闲置：关闭）')

    # ------------------------------------------------------------------
    # 内部回调
    # ------------------------------------------------------------------

    def _on_angles_update(self, angles):
        """EyeController 更新角度时回调，缓存供定时器发布。"""
        self._current_angles = angles

    def _publish_timer(self):
        msg = Float32MultiArray()
        msg.data = [float(a) for a in self._current_angles]
        self._pub.publish(msg)

    # ------------------------------------------------------------------
    # 话题回调
    # ------------------------------------------------------------------

    def _gaze_cb(self, msg: Float32MultiArray):
        """
        接收凝视指令: [pitch, yaw] 或 [pitch, yaw, duration_ms]
        pitch: -1(下)~+1(上), yaw: -1(右)~+1(左)
        """
        data = list(msg.data)
        if len(data) < 2:
            self.get_logger().warn('gaze_cmd 需要至少2个值 [pitch, yaw]')
            return
        pitch = float(data[0])
        yaw   = float(data[1])
        dur   = float(data[2]) if len(data) > 2 else 150.0
        threading.Thread(
            target=self._eye.set_gaze, args=(pitch, yaw, dur), daemon=True
        ).start()
        self.get_logger().debug(f'凝视指令: pitch={pitch:.2f} yaw={yaw:.2f} dur={dur}ms')

    # ------------------------------------------------------------------
    # 服务回调
    # ------------------------------------------------------------------

    def _blink_cb(self, _req, response):
        threading.Thread(target=self._eye.blink, daemon=True).start()
        response.success = True
        response.message = 'Blink triggered'
        return response

    def _wink_cb(self, _req, response):
        threading.Thread(
            target=self._eye.wink, args=('right',), daemon=True
        ).start()
        response.success = True
        response.message = 'Wink triggered'
        return response

    def _center_cb(self, _req, response):
        threading.Thread(target=self._eye.center_gaze, daemon=True).start()
        response.success = True
        response.message = 'Gaze centered'
        return response

    def _idle_on_cb(self, _req, response):
        self._eye.start_idle()
        self.get_logger().info('自动闲置已开启')
        response.success = True
        response.message = 'Idle started'
        return response

    def _idle_off_cb(self, _req, response):
        self._eye.stop_idle()
        self.get_logger().info('自动闲置已关闭')
        response.success = True
        response.message = 'Idle stopped'
        return response

    def destroy_node(self):
        self._eye.stop_idle()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EyeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

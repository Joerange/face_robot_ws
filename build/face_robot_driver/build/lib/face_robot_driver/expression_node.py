"""
ROS 2 表情控制节点
服务: /face/play_expression (name, duration_ms)
话题: /face/servo_angles (输出到驱动节点)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger
import os

from .expression_player import ExpressionPlayer
from .pom_sc32 import NUM_SERVOS


class ExpressionNode(Node):

    def __init__(self):
        super().__init__('expression_node')

        self.declare_parameter('expressions_yaml',
                               os.path.expanduser('~/Mark/face_robot_ws/config/expressions.yaml'))

        yaml_path = self.get_parameter('expressions_yaml').value

        self._pub = self.create_publisher(Float32MultiArray, '/face/servo_angles', 10)

        self._player = ExpressionPlayer(
            config_path=yaml_path,
            on_angles_update=self._publish_angles
        )

        # 通过话题接收表情名称（便于快速测试）
        self._sub = self.create_subscription(
            String, '/face/expression_cmd', self._expression_cb, 10
        )

        self.get_logger().info(
            f'表情节点已启动，已加载表情: {self._player.expression_names}'
        )

    def _expression_cb(self, msg: String):
        parts = msg.data.strip().split(',')
        name = parts[0].strip()
        duration = float(parts[1]) if len(parts) > 1 else 500.0

        ok = self._player.play(name, duration_ms=duration, interp='cubic')
        if ok:
            self.get_logger().info(f'播放表情: {name}  时长={duration}ms')
        else:
            self.get_logger().warn(f'未知表情: {name}  可用: {self._player.expression_names}')

    def _publish_angles(self, angles: list):
        msg = Float32MultiArray()
        msg.data = [float(a) for a in angles]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ExpressionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

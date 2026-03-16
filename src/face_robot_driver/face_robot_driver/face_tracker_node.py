"""
人脸追踪节点

使用 OpenCV Haar 级联分类器检测人脸中心，
通过比例控制器驱动眼球舵机(id13/id14)追踪人脸，
眼睑耦合由 EyeController 自动处理。

机器人无颈部舵机，追踪范围为眼球运动极限。

话题（发布）:
  /face/servo_angles   Float32MultiArray   32路舵机角度

参数:
  camera_index    int    摄像头编号（默认0）
  servos_yaml     str    servos.yaml 路径
  kp              float  比例增益，每帧步长系数（默认0.08）
  deadzone_px     int    像素级死区，小于此误差不动（默认30）
  idle_timeout    float  无人脸N秒后进入自动闲置（默认3.0）
  show_preview    bool   是否显示摄像头预览窗口（默认False）
"""

import os
import time
import threading

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from .servo_config import ServoConfig
from .eye_controller import EyeController


class FaceTrackerNode(Node):

    def __init__(self):
        super().__init__('face_tracker')

        # --- 参数声明 ---
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('servos_yaml',
            os.path.expanduser('~/Mark/face_robot_ws/config/servos.yaml'))
        self.declare_parameter('kp', 0.08)
        self.declare_parameter('deadzone_px', 30)
        self.declare_parameter('idle_timeout', 3.0)
        self.declare_parameter('show_preview', True)

        cam_idx      = self.get_parameter('camera_index').value
        yaml_path    = self.get_parameter('servos_yaml').value
        self._kp     = self.get_parameter('kp').value
        self._dz_px  = self.get_parameter('deadzone_px').value
        self._idle_t = self.get_parameter('idle_timeout').value
        self._preview= self.get_parameter('show_preview').value

        # --- 发布（必须在 EyeController 之前，初始化时会立即触发回调）---
        self._pub = self.create_publisher(Float32MultiArray, '/face/servo_angles', 10)

        # --- 舵机配置 + 眼部控制器 ---
        cfg = ServoConfig(yaml_path)
        self._current_angles = cfg.neutral_angles()
        self._eye = EyeController(cfg, on_update=self._on_angles_update)

        # --- 追踪状态 ---
        self._target_pitch = 0.0
        self._target_yaw   = 0.0
        self._last_face_t  = time.time()
        self._idle_active  = False
        self._state_lock   = threading.Lock()

        # --- 预览帧（主线程显示用）---
        self._preview_frame = None
        self._frame_lock    = threading.Lock()

        # --- Haar 级联分类器 ---
        cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        self._cascade = cv2.CascadeClassifier(cascade_path)
        if self._cascade.empty():
            raise RuntimeError(f'无法加载 Haar 级联文件: {cascade_path}')

        # --- 摄像头 ---
        self._cap = cv2.VideoCapture(cam_idx)
        if not self._cap.isOpened():
            raise RuntimeError(f'无法打开摄像头 index={cam_idx}')

        self._frame_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self._frame_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._cx = self._frame_w // 2
        self._cy = self._frame_h // 2

        self.get_logger().info(
            f'摄像头已打开: index={cam_idx}  分辨率={self._frame_w}x{self._frame_h}')

        # --- 摄像头线程 ---
        self._stop_event = threading.Event()
        self._cam_thread = threading.Thread(
            target=self._camera_loop, daemon=True, name='face_tracker_cam')
        self._cam_thread.start()

        # --- 预览定时器（主线程刷新 OpenCV 窗口）---
        if self._preview:
            self.create_timer(0.033, self._preview_timer)   # ~30Hz

        self.get_logger().info(
            f'人脸追踪节点已启动  kp={self._kp}  deadzone={self._dz_px}px'
            f'  idle_timeout={self._idle_t}s')

    # ------------------------------------------------------------------
    # 角度回调 → 发布
    # ------------------------------------------------------------------

    def _on_angles_update(self, angles):
        self._current_angles = angles
        msg = Float32MultiArray()
        msg.data = [float(a) for a in angles]
        self._pub.publish(msg)

    # ------------------------------------------------------------------
    # 摄像头检测线程（只做检测和控制，不调 imshow）
    # ------------------------------------------------------------------

    def _camera_loop(self):
        while not self._stop_event.is_set():
            ret, frame = self._cap.read()
            if not ret:
                self.get_logger().warn('摄像头读帧失败，重试...')
                time.sleep(0.05)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self._cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(60, 60)
            )

            face_detected = len(faces) > 0
            if face_detected:
                # 取面积最大的人脸
                x, y, w, h = max(faces, key=lambda f: f[2] * f[3])
                face_x = x + w // 2
                face_y = y + h // 2

                err_x = self._cx - face_x   # 正 = 人脸在画面左侧
                err_y = self._cy - face_y   # 正 = 人脸在画面上方

                self._update_gaze(err_x, err_y)

                if self._preview:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(frame, (face_x, face_y), 5, (0, 255, 0), -1)
                    with self._state_lock:
                        yaw   = self._target_yaw
                        pitch = self._target_pitch
                    cv2.putText(
                        frame,
                        f'err({err_x:+d},{err_y:+d}) '
                        f'yaw={yaw:+.2f} pitch={pitch:+.2f}',
                        (10, self._frame_h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

            if self._preview:
                cv2.drawMarker(frame, (self._cx, self._cy),
                               (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
                with self._frame_lock:
                    self._preview_frame = frame

            self._handle_idle(face_detected)

    # ------------------------------------------------------------------
    # 预览窗口定时器（主线程，imshow 必须在主线程）
    # ------------------------------------------------------------------

    def _preview_timer(self):
        with self._frame_lock:
            frame = self._preview_frame
        if frame is not None:
            cv2.imshow('Face Tracker', frame)
            cv2.waitKey(1)

    # ------------------------------------------------------------------
    # 比例控制器
    # ------------------------------------------------------------------

    def _update_gaze(self, err_x: int, err_y: int):
        """
        比例控制：根据像素误差增量更新目标 pitch/yaw。

        err_x 正 = 人脸在画面左侧 → yaw 增大（眼球转左）
        err_y 正 = 人脸在画面上方 → pitch 增大（眼球转上）
        死区内不响应，避免小幅抖动。
        """
        norm_x = err_x / max(self._cx, 1)
        norm_y = err_y / max(self._cy, 1)

        delta_yaw   = self._kp * norm_x if abs(err_x) > self._dz_px else 0.0
        delta_pitch = self._kp * norm_y if abs(err_y) > self._dz_px else 0.0

        with self._state_lock:
            self._target_yaw   = max(-1.0, min(1.0, self._target_yaw   + delta_yaw))
            self._target_pitch = max(-1.0, min(1.0, self._target_pitch + delta_pitch))
            yaw   = self._target_yaw
            pitch = self._target_pitch

        self._eye.set_gaze_instant(pitch, yaw)
        self._last_face_t = time.time()

    # ------------------------------------------------------------------
    # 自动闲置管理
    # ------------------------------------------------------------------

    def _handle_idle(self, face_detected: bool):
        if face_detected:
            if self._idle_active:
                self._eye.stop_idle()
                self._idle_active = False
                self.get_logger().info('检测到人脸，停止闲置模式')
        else:
            if not self._idle_active and (time.time() - self._last_face_t) > self._idle_t:
                self._eye.start_idle()
                self._idle_active = True
                self.get_logger().info('无人脸超时，进入闲置模式')

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        self._stop_event.set()
        self._eye.stop_idle()
        self._cap.release()
        if self._preview:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FaceTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

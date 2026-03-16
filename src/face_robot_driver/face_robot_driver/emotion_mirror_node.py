"""
情绪镜像节点

摄像头检测人脸情绪 → 机器人跟随做出相同表情。

流程:
  摄像头帧 → FaceLandmarker(blendshapes) → 情绪分类 → EMA滤波
  → 表情切换时 smoothstep 过渡(0.6s) → 发布 /face/servo_angles

只控制表情相关舵机(1-8, 15-22, 28-30)，
眼部舵机(9-14)保持 neutral。

话题（发布）:
  /face/servo_angles      Float32MultiArray  32路角度
  /face/detected_emotion  String             当前识别情绪（调试用）

参数:
  camera_index           int    摄像头编号（默认0）
  model_path             str    FaceLandmarker 模型路径
  servos_yaml            str    servos.yaml 路径
  ema_alpha              float  EMA 平滑系数（默认0.3）
  min_hold_s             float  情绪最小保持时间（默认0.5）
  confidence_threshold   float  切换阈值（默认0.25）
  transition_duration    float  表情过渡时长秒（默认0.6）
  show_preview           bool   显示预览窗口（默认True）
"""

import os
import time
import threading

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from .servo_config import ServoConfig
from .expressions import EXPRESSIONS, EXPRESSION_SERVO_IDS, smoothstep
from .emotion_detector import EmotionDetector, draw_face_mesh


class EmotionMirrorNode(Node):

    def __init__(self):
        super().__init__('emotion_mirror')

        # --- 参数 ---
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('model_path', os.path.expanduser(
            '~/Mark/face_robot_ws/models/face_landmarker.task'))
        self.declare_parameter('servos_yaml', os.path.expanduser(
            '~/Mark/face_robot_ws/config/servos.yaml'))
        self.declare_parameter('ema_alpha', 0.3)
        self.declare_parameter('min_hold_s', 0.5)
        self.declare_parameter('confidence_threshold', 0.08)
        self.declare_parameter('transition_duration', 0.6)
        self.declare_parameter('show_preview', True)

        cam_idx    = self.get_parameter('camera_index').value
        model_path = self.get_parameter('model_path').value
        yaml_path  = self.get_parameter('servos_yaml').value
        ema_alpha  = self.get_parameter('ema_alpha').value
        min_hold   = self.get_parameter('min_hold_s').value
        threshold  = self.get_parameter('confidence_threshold').value
        self._trans_dur = self.get_parameter('transition_duration').value
        self._preview   = self.get_parameter('show_preview').value

        # --- 发布 ---
        self._pub_angles  = self.create_publisher(Float32MultiArray, '/face/servo_angles', 10)
        self._pub_emotion = self.create_publisher(String, '/face/detected_emotion', 10)

        # --- 舵机配置 ---
        self._cfg = ServoConfig(yaml_path)
        self._neutral = self._cfg.neutral_angles()

        # --- 当前角度状态 ---
        self._current_angles = list(self._neutral)
        self._target_angles  = list(self._neutral)
        self._transitioning  = False
        self._trans_start    = 0.0
        self._trans_from     = list(self._neutral)
        self._angles_lock    = threading.Lock()

        # --- 当前情绪 ---
        self._current_emotion = 'neutral'

        # --- 情绪检测器 ---
        self._detector = EmotionDetector(
            model_path, ema_alpha, min_hold, threshold)

        # --- 摄像头 ---
        self._cap = cv2.VideoCapture(cam_idx)
        if not self._cap.isOpened():
            raise RuntimeError(f'无法打开摄像头 index={cam_idx}')
        self._frame_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self._frame_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            f'摄像头已打开: index={cam_idx}  分辨率={self._frame_w}x{self._frame_h}')

        # --- 预览帧 ---
        self._preview_frame = None
        self._frame_lock = threading.Lock()

        # --- 摄像头检测线程 ---
        self._stop_event = threading.Event()
        self._cam_thread = threading.Thread(
            target=self._camera_loop, daemon=True, name='emotion_cam')
        self._cam_thread.start()

        # --- 过渡 + 发布定时器 (50Hz) ---
        self.create_timer(0.02, self._control_timer)

        # --- 预览定时器 (主线程 imshow) ---
        if self._preview:
            self.create_timer(0.033, self._preview_timer)

        self.get_logger().info(
            f'情绪镜像节点已启动  ema={ema_alpha}  hold={min_hold}s  '
            f'threshold={threshold}  transition={self._trans_dur}s')

    # ------------------------------------------------------------------
    # 摄像头检测线程
    # ------------------------------------------------------------------

    def _camera_loop(self):
        start_time = time.time()

        while not self._stop_event.is_set():
            ret, frame = self._cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            # 计算时间戳 (ms)
            ts_ms = int((time.time() - start_time) * 1000)

            emotion, raw_scores, bs, landmarks = self._detector.detect(frame, ts_ms)

            # 情绪变化 → 启动过渡
            if emotion != self._current_emotion:
                self._start_transition(emotion)
                self._current_emotion = emotion

                # 发布情绪话题
                msg = String()
                msg.data = emotion
                self._pub_emotion.publish(msg)
                self.get_logger().info(f'表情切换 → {emotion}')

            # 预览帧
            if self._preview:
                self._draw_preview(frame, emotion, raw_scores, landmarks)

            # ~15Hz 检测节奏
            time.sleep(0.06)

    # ------------------------------------------------------------------
    # 表情过渡
    # ------------------------------------------------------------------

    def _start_transition(self, emotion: str):
        """启动 smoothstep 过渡到新表情。"""
        overrides = EXPRESSIONS.get(emotion, {})

        # 目标：neutral 基础 + 表情覆盖
        target = list(self._neutral)
        for sid, angle in overrides.items():
            target[sid - 1] = angle
        target = self._cfg.apply(target)

        with self._angles_lock:
            self._trans_from  = list(self._current_angles)
            self._target_angles = target
            self._trans_start = time.time()
            self._transitioning = True

    def _control_timer(self):
        """50Hz 定时器：执行过渡插值 + 发布角度。"""
        with self._angles_lock:
            if self._transitioning:
                elapsed = time.time() - self._trans_start
                t = min(1.0, elapsed / max(self._trans_dur, 0.01))
                t = smoothstep(t)

                for i in range(len(self._current_angles)):
                    sid = i + 1
                    if sid in EXPRESSION_SERVO_IDS:
                        self._current_angles[i] = (
                            self._trans_from[i]
                            + (self._target_angles[i] - self._trans_from[i]) * t
                        )

                if t >= 1.0:
                    self._transitioning = False

            angles = list(self._current_angles)

        msg = Float32MultiArray()
        msg.data = [float(a) for a in angles]
        self._pub_angles.publish(msg)

    # ------------------------------------------------------------------
    # 预览
    # ------------------------------------------------------------------

    def _draw_preview(self, frame, emotion: str, raw_scores: dict, landmarks=None):
        ema = self._detector.ema_scores

        # 绘制 face mesh 网格
        if landmarks is not None:
            draw_face_mesh(frame, landmarks, self._frame_w, self._frame_h)

        # 情绪标签（保留用户设置的黑色字体）
        color = {'happy': (0,0,0), 'angry': (0,0,0),
                 'sad': (0,0,0), 'neutral': (0,0,0)}.get(emotion, (255,255,255))
        cv2.putText(frame, f'Emotion: {emotion}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)

        # EMA 分数条
        y = 60
        for e in ('happy', 'angry', 'sad', 'neutral'):
            score = ema.get(e, 0)
            bar_w = int(score * 200)
            c = (0,200,0) if e == emotion else (150,150,150)
            cv2.rectangle(frame, (10, y), (10 + bar_w, y + 16), c, -1)
            cv2.putText(frame, f'{e}: {score:.2f}',
                        (220, y + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,0,0), 1)
            y += 22

        with self._frame_lock:
            self._preview_frame = frame

    def _preview_timer(self):
        with self._frame_lock:
            frame = self._preview_frame
        if frame is not None:
            cv2.imshow('Emotion Mirror', frame)
            cv2.waitKey(1)

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        self._stop_event.set()
        self._cap.release()
        self._detector.close()
        if self._preview:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EmotionMirrorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

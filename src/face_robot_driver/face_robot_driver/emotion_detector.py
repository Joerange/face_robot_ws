"""
情绪检测器

使用 MediaPipe FaceLandmarker (Tasks API) 提取 52 个 blendshape 系数，
通过加权规则将 blendshapes 映射到 4 种情绪 (happy/angry/sad/neutral)，
并用 EMA + 滞回滤波防止频繁切换。
"""

import time
import logging

import cv2
import mediapipe as mp

logger = logging.getLogger(__name__)

_BaseOptions = mp.tasks.BaseOptions
_FaceLandmarker = mp.tasks.vision.FaceLandmarker
_FaceLandmarkerOptions = mp.tasks.vision.FaceLandmarkerOptions
_RunningMode = mp.tasks.vision.RunningMode

# Face mesh 连接线（用于绘制网格）
_FaceConns = mp.tasks.vision.FaceLandmarksConnections
FACE_TESSELATION = _FaceConns.FACE_LANDMARKS_TESSELATION
FACE_CONTOURS    = _FaceConns.FACE_LANDMARKS_CONTOURS

EMOTIONS = ('happy', 'angry', 'sad', 'neutral')


class EmotionClassifier:
    """从 blendshape 系数分类情绪。"""

    @staticmethod
    def classify(bs: dict) -> dict:
        """
        返回各情绪的原始打分 dict。

        核心问题: angry/sad 相关 blendshape 天然数值偏低 (0.1~0.3),
        而 smile 轻松达到 0.5~0.9。因此对弱信号施加增益放大。
        """
        # --- 提取信号 ---
        smile       = (bs.get('mouthSmileLeft', 0) + bs.get('mouthSmileRight', 0)) / 2
        cheek       = (bs.get('cheekSquintLeft', 0) + bs.get('cheekSquintRight', 0)) / 2
        frown       = (bs.get('mouthFrownLeft', 0) + bs.get('mouthFrownRight', 0)) / 2
        brow_down   = (bs.get('browDownLeft', 0) + bs.get('browDownRight', 0)) / 2
        brow_up     = bs.get('browInnerUp', 0)
        brow_outer  = (bs.get('browOuterUpLeft', 0) + bs.get('browOuterUpRight', 0)) / 2
        nose_sneer  = (bs.get('noseSneerLeft', 0) + bs.get('noseSneerRight', 0)) / 2
        mouth_press = (bs.get('mouthPressLeft', 0) + bs.get('mouthPressRight', 0)) / 2
        eye_squint  = (bs.get('eyeSquintLeft', 0) + bs.get('eyeSquintRight', 0)) / 2
        mouth_pucker= bs.get('mouthPucker', 0)
        lip_stretch = (bs.get('mouthStretchLeft', 0) + bs.get('mouthStretchRight', 0)) / 2
        mouth_lower = (bs.get('mouthLowerDownLeft', 0) + bs.get('mouthLowerDownRight', 0)) / 2

        # --- 打分（对弱信号类别施加增益）---
        happy_raw = (0.45 * smile
                     + 0.30 * cheek
                     + 0.15 * eye_squint
                     + 0.10 * max(0, brow_up - 0.1))

        angry_raw = (0.28 * brow_down
                     + 0.22 * nose_sneer
                     + 0.18 * mouth_press
                     + 0.15 * eye_squint
                     + 0.10 * frown
                     + 0.07 * lip_stretch)

        sad_raw   = (0.30 * frown
                     + 0.25 * brow_up
                     + 0.15 * mouth_pucker
                     + 0.15 * mouth_lower
                     + 0.15 * max(0, 0.2 - smile))

        # 增益放大弱信号类别，使其分值与 happy 可比
        scores = {
            'happy':   happy_raw,
            'angry':   angry_raw * 2.5,
            'sad':     sad_raw   * 1.0,
            'neutral': 0.3,
        }

        # 存在明显信号时进一步压低 neutral
        max_signal = max(smile, frown, brow_down, nose_sneer, mouth_press)
        if max_signal > 0.15:
            scores['neutral'] *= max(0.0, 1.0 - max_signal * 2)

        return scores


class EmotionFilter:
    """
    EMA 平滑 + 最小保持时间 + 滞回阈值，防止情绪频繁跳变。
    """

    def __init__(self, ema_alpha: float = 0.3,
                 min_hold_s: float = 0.5,
                 threshold: float = 0.08):
        self._alpha = ema_alpha
        self._min_hold = min_hold_s
        self._threshold = threshold
        self._ema = {e: (0.15 if e == 'neutral' else 0.0) for e in EMOTIONS}
        self._current = 'neutral'
        self._hold_until = 0.0

    @property
    def current(self) -> str:
        return self._current

    @property
    def ema_scores(self) -> dict:
        return dict(self._ema)

    def update(self, raw_scores: dict) -> str:
        """输入原始分数，返回滤波后的情绪名。"""
        now = time.time()

        for e in EMOTIONS:
            self._ema[e] = (self._alpha * raw_scores.get(e, 0.0)
                            + (1.0 - self._alpha) * self._ema[e])

        best = max(self._ema, key=self._ema.get)

        if best != self._current and now >= self._hold_until:
            margin = self._ema[best] - self._ema[self._current]
            if margin > self._threshold:
                self._current = best
                self._hold_until = now + self._min_hold
                logger.info(f'情绪切换 → {best} (margin={margin:.3f})')

        return self._current


class EmotionDetector:
    """
    封装 FaceLandmarker + 分类 + 滤波。

    detect() 返回 (emotion, scores, blendshapes, landmarks)
    其中 landmarks 可用于绘制 face mesh。
    """

    def __init__(self, model_path: str,
                 ema_alpha: float = 0.3,
                 min_hold_s: float = 0.5,
                 threshold: float = 0.08):

        options = _FaceLandmarkerOptions(
            base_options=_BaseOptions(model_asset_path=model_path),
            running_mode=_RunningMode.VIDEO,
            num_faces=1,
            min_face_detection_confidence=0.5,
            min_face_presence_confidence=0.5,
            min_tracking_confidence=0.5,
            output_face_blendshapes=True,
        )
        self._landmarker = _FaceLandmarker.create_from_options(options)
        self._classifier = EmotionClassifier()
        self._filter = EmotionFilter(ema_alpha, min_hold_s, threshold)

    def detect(self, bgr_frame, timestamp_ms: int) -> tuple:
        """
        处理一帧 BGR 图像。

        返回: (emotion, raw_scores, blendshapes_dict, landmarks_list)
               landmarks_list: [{x, y, z}, ...] 478个归一化坐标，或 None
        """
        rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        result = self._landmarker.detect_for_video(mp_image, timestamp_ms)

        if not result.face_landmarks:
            return self._filter.current, {}, None, None

        # landmarks (478 个归一化坐标)
        landmarks = result.face_landmarks[0]

        # blendshapes
        bs = None
        if result.face_blendshapes:
            bs = {cat.category_name: cat.score
                  for cat in result.face_blendshapes[0]}

        if bs:
            raw_scores = self._classifier.classify(bs)
            emotion = self._filter.update(raw_scores)
        else:
            raw_scores = {}
            emotion = self._filter.current

        return emotion, raw_scores, bs, landmarks

    @property
    def current_emotion(self) -> str:
        return self._filter.current

    @property
    def ema_scores(self) -> dict:
        return self._filter.ema_scores

    def close(self):
        self._landmarker.close()


def draw_face_mesh(frame, landmarks, w: int, h: int):
    """
    在帧上绘制 face mesh 网格线 + 轮廓线。

    landmarks: MediaPipe NormalizedLandmark 列表 (478个)
    w, h: 帧像素尺寸
    """
    # 转为像素坐标
    pts = [(int(lm.x * w), int(lm.y * h)) for lm in landmarks]

    # 绘制 tesselation 网格（浅灰色细线）
    for conn in FACE_TESSELATION:
        p1 = pts[conn.start]
        p2 = pts[conn.end]
        cv2.line(frame, p1, p2, (100, 100, 100), 1, cv2.LINE_AA)

    # 绘制轮廓（绿色，稍粗）
    for conn in FACE_CONTOURS:
        p1 = pts[conn.start]
        p2 = pts[conn.end]
        cv2.line(frame, p1, p2, (0, 200, 0), 1, cv2.LINE_AA)

"""
眼部运动控制器
参考: EyeMech ε3.2 (Will Cogley)

舵机映射:
  id9  right_eyelid_upper  dir=+1  angle↑=张开
  id10 right_eyelid_lower  dir=+1  angle↑=张开
  id11 left_eyelid_upper   dir=-1  angle↑=关闭
  id12 left_eyelid_lower   dir=-1  angle↑=关闭
  id13 eyeball_pitch       dir=+1  angle↑=向上, range=[50,90], neutral=90
  id14 eyeball_yaw         dir=+1  angle↑=向左, range=[60,100], neutral=80

坐标约定 (归一化):
  pitch: -1=向下, +1=向上  (受物理限制，该机构只能向下)
  yaw:   -1=向右, +1=向左  (机器人视角)
  lid_open: 0=完全闭合, 1=完全张开
"""

import time
import random
import threading
from typing import Optional, List

from .servo_config import ServoConfig, NUM_SERVOS

# 眼部舵机 ID（1-indexed）
_RU = 9    # right_eyelid_upper
_RL = 10   # right_eyelid_lower
_LU = 11   # left_eyelid_upper
_LL = 12   # left_eyelid_lower
_PT = 13   # eyeball_pitch
_YW = 14   # eyeball_yaw

_LID_IDS = [_RU, _RL, _LU, _LL]
_EYE_IDS = [_RU, _RL, _LU, _LL, _PT, _YW]

# 眼睑跟随眼球俯仰的耦合系数 (参考 EyeMech: upper=0.8, lower=0.4，本机适当减小)
_UPPER_COUPLING = 0.35
_LOWER_COUPLING = 0.18


class EyeController:
    """
    管理6路眼部舵机，提供：
    - 平滑凝视（含眼睑俯仰耦合）
    - 自然眨眼（快合慢开）
    - 自动闲置（随机扫视 + 周期眨眼）
    """

    def __init__(self, servo_config: ServoConfig, on_update=None):
        """
        servo_config: ServoConfig 实例
        on_update: callable(angles: list[float])，每次角度更新时回调
        """
        self._cfg = servo_config
        self._on_update = on_update
        self._lock = threading.Lock()

        # 当前状态（归一化）
        self._pitch: float = 0.0
        self._yaw: float = 0.0
        self._lid_open: float = 1.0  # 0=闭合, 1=张开
        self._blinking: bool = False

        # 32路角度，初始化为中位
        self._angles: List[float] = self._cfg.neutral_angles()
        # 初始化时将眼睑设为张开状态
        self._push_eye_angles(self._compute_angles(0.0, 0.0, 1.0))

        # 自动闲置线程
        self._idle_thread: Optional[threading.Thread] = None
        self._idle_stop = threading.Event()

    # ------------------------------------------------------------------
    # 角度计算
    # ------------------------------------------------------------------

    def _lid_angle(self, sid: int, open_frac: float) -> float:
        """将 open_frac [0=闭, 1=开] 转换为指定眼睑舵机的原始角度。"""
        p = self._cfg.get(sid)
        f = max(0.0, min(1.0, open_frac))
        if p.direction == 1:   # 角度增大 = 张开
            return p.min + f * (p.max - p.min)
        else:                  # 角度增大 = 关闭
            return p.max - f * (p.max - p.min)

    def _pitch_angle(self, pitch_norm: float) -> float:
        """pitch_norm [-1=下, +1=上] → 眼球俯仰原始角度。"""
        p = self._cfg.get(_PT)
        if pitch_norm >= 0:
            # 向上：neutral → max（物理上几乎没有余量）
            return p.neutral + pitch_norm * (p.max - p.neutral)
        else:
            # 向下：neutral → min
            return p.neutral + pitch_norm * (p.neutral - p.min)

    def _yaw_angle(self, yaw_norm: float) -> float:
        """yaw_norm [-1=右, +1=左] → 眼球偏航原始角度。"""
        p = self._cfg.get(_YW)
        if yaw_norm >= 0:
            return p.neutral + yaw_norm * (p.max - p.neutral)
        else:
            return p.neutral + yaw_norm * (p.neutral - p.min)

    def _compute_angles(self, pitch: float, yaw: float,
                        lid_open: float) -> dict:
        """
        计算6路眼部舵机角度。
        返回 {servo_id: angle}。

        眼睑耦合：眼球向下看时，上眼睑略微下垂（自然感）。
        coupling 仅在 pitch < 0（向下）时生效，向上无效（物理限制）。
        """
        pitch_factor = min(0.0, pitch)   # 只在向下时耦合
        upper_frac = lid_open + pitch_factor * _UPPER_COUPLING
        lower_frac = lid_open + pitch_factor * _LOWER_COUPLING

        return {
            _RU: self._lid_angle(_RU, upper_frac),
            _RL: self._lid_angle(_RL, lower_frac),
            _LU: self._lid_angle(_LU, upper_frac),
            _LL: self._lid_angle(_LL, lower_frac),
            _PT: self._pitch_angle(pitch),
            _YW: self._yaw_angle(yaw),
        }

    def _push_eye_angles(self, eye_angles: dict):
        """将眼部角度写入32路数组并触发回调。"""
        with self._lock:
            for sid, angle in eye_angles.items():
                self._angles[sid - 1] = angle
            snapshot = list(self._angles)
        if self._on_update:
            self._on_update(snapshot)

    # ------------------------------------------------------------------
    # 凝视控制
    # ------------------------------------------------------------------

    def set_gaze(self, pitch: float, yaw: float, duration_ms: float = 150.0):
        """
        平滑移动眼球至 (pitch, yaw)，同时驱动眼睑耦合。

        pitch: -1(下) ~ +1(上)
        yaw:   -1(右) ~ +1(左)
        duration_ms: 过渡时长(ms)，100-300ms 为自然扫视范围
        """
        pitch = max(-1.0, min(1.0, float(pitch)))
        yaw   = max(-1.0, min(1.0, float(yaw)))

        start_pitch, start_yaw = self._pitch, self._yaw

        dt = 0.018   # ~55Hz
        steps = max(1, int(duration_ms / 1000.0 / dt))

        for i in range(steps + 1):
            t = i / steps
            t = t * t * (3 - 2 * t)   # smoothstep：起止平滑

            cur_pitch = start_pitch + (pitch - start_pitch) * t
            cur_yaw   = start_yaw   + (yaw   - start_yaw)   * t

            self._push_eye_angles(
                self._compute_angles(cur_pitch, cur_yaw, self._lid_open)
            )
            if i < steps:
                time.sleep(dt)

        self._pitch = pitch
        self._yaw   = yaw

    def set_gaze_instant(self, pitch: float, yaw: float) -> None:
        """
        立即将眼球设置到目标位置（无动画），用于实时追踪。
        线程安全：可在摄像头线程中频繁调用。
        """
        pitch = max(-1.0, min(1.0, float(pitch)))
        yaw   = max(-1.0, min(1.0, float(yaw)))
        self._pitch = pitch
        self._yaw   = yaw
        self._push_eye_angles(self._compute_angles(pitch, yaw, self._lid_open))

    def center_gaze(self, duration_ms: float = 200.0):
        """眼球回正中。"""
        self.set_gaze(0.0, 0.0, duration_ms)

    # ------------------------------------------------------------------
    # 眨眼
    # ------------------------------------------------------------------

    def blink(self, half: bool = False):
        """
        自然眨眼：快速闭合 → 短暂停留 → 缓慢张开。

        half=True: 半眨眼（困倦感，闭合至约50%）
        """
        if self._blinking:
            return
        self._blinking = True
        try:
            close_target = 0.0 if not half else 0.50
            self._animate_lids(close_target, duration_ms=80)   # 快合
            time.sleep(0.035)                                   # 停留
            self._animate_lids(1.0, duration_ms=130)           # 慢开
        finally:
            self._blinking = False

    def wink(self, side: str = 'right'):
        """
        单眼眨眼（wink）。
        side: 'right' 或 'left'
        """
        if side == 'right':
            lids = [_RU, _RL]
        else:
            lids = [_LU, _LL]

        self._animate_lids(0.0, duration_ms=80, lids=lids)
        time.sleep(0.05)
        self._animate_lids(1.0, duration_ms=130, lids=lids)

    def open_eyes(self, duration_ms: float = 200.0):
        """完全张开眼睛。"""
        self._animate_lids(1.0, duration_ms)

    def close_eyes(self, duration_ms: float = 250.0):
        """完全闭合眼睛（如睡眠表情）。"""
        self._animate_lids(0.0, duration_ms)

    def _animate_lids(self, target_open: float, duration_ms: float,
                      lids: Optional[list] = None):
        """
        平滑驱动眼睑到 target_open 分数。
        lids: None=全部4个眼睑, 否则指定 servo_id 列表
        """
        start_open = self._lid_open
        dt = 0.012   # ~83Hz，眨眼需要更高频率
        steps = max(2, int(duration_ms / 1000.0 / dt))

        for i in range(steps + 1):
            t = i / steps
            cur_open = start_open + (target_open - start_open) * t
            self._lid_open = cur_open

            if lids is None:
                # 全部眼睑，带耦合
                self._push_eye_angles(
                    self._compute_angles(self._pitch, self._yaw, cur_open)
                )
            else:
                # 仅指定眼睑，不影响眼球
                partial = {sid: self._lid_angle(sid, cur_open) for sid in lids}
                self._push_eye_angles(partial)

            if i < steps:
                time.sleep(dt)

        self._lid_open = target_open

    # ------------------------------------------------------------------
    # 自动闲置行为
    # ------------------------------------------------------------------

    def start_idle(self):
        """
        启动自动闲置线程。
        行为：周期性眨眼(3-7s) + 随机扫视(1-4s)，模拟自然眼动。
        """
        if self._idle_thread and self._idle_thread.is_alive():
            return
        self._idle_stop.clear()
        self._idle_thread = threading.Thread(
            target=self._idle_loop, daemon=True, name='eye_idle'
        )
        self._idle_thread.start()

    def stop_idle(self, timeout: float = 1.0):
        """停止自动闲置线程。"""
        self._idle_stop.set()
        if self._idle_thread:
            self._idle_thread.join(timeout=timeout)

    def _idle_loop(self):
        """
        闲置行为主循环（参考 EyeMech 的 auto mode）。

        策略:
          - 微扫视（60%）: 幅度小, 快速, 模拟注视点漂移
          - 大扫视（40%）: 幅度大, 模拟视线转移
          - 偶发双眨眼（20%）
        """
        next_blink   = time.time() + random.uniform(3.0, 7.0)
        next_saccade = time.time() + random.uniform(0.8, 2.5)

        while not self._idle_stop.is_set():
            now = time.time()

            # --- 眨眼 ---
            if now >= next_blink:
                self.blink()
                if random.random() < 0.2:       # 20%：双眨眼
                    time.sleep(random.uniform(0.12, 0.28))
                    self.blink()
                next_blink = now + random.uniform(3.0, 7.0)

            # --- 扫视 ---
            if now >= next_saccade and not self._blinking:
                if random.random() < 0.6:       # 微扫视
                    pitch = random.uniform(-0.25, 0.0)
                    yaw   = random.uniform(-0.25, 0.25)
                    dur   = random.uniform(80, 160)
                else:                            # 大扫视
                    pitch = random.uniform(-0.8, 0.0)
                    yaw   = random.uniform(-0.7, 0.7)
                    dur   = random.uniform(160, 320)

                self.set_gaze(pitch, yaw, dur)
                next_saccade = now + random.uniform(1.0, 4.0)

            time.sleep(0.05)

    # ------------------------------------------------------------------
    # 工具
    # ------------------------------------------------------------------

    def get_angles(self) -> List[float]:
        """返回当前32路角度快照。"""
        with self._lock:
            return list(self._angles)

    @property
    def state(self) -> dict:
        """返回当前眼部状态（调试用）。"""
        return {
            'pitch': round(self._pitch, 3),
            'yaw':   round(self._yaw, 3),
            'lid_open': round(self._lid_open, 3),
        }

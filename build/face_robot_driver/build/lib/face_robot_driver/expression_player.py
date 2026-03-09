"""
表情播放器
- 加载 YAML 表情库
- 支持线性插值过渡
- 可中断/切换
"""

import yaml
import time
import threading
import math
import os
from typing import Optional

from .pom_sc32 import NUM_SERVOS


class ExpressionPlayer:
    """
    基于角度插值的表情播放器。
    与 ServoDriverNode 解耦，通过回调函数输出目标角度。
    """

    def __init__(self, config_path: str, on_angles_update=None):
        """
        config_path: expressions.yaml 路径
        on_angles_update: callable(angles: list[float]) 角度更新回调
        """
        self._expressions: dict = {}
        self._on_update = on_angles_update
        self._current_angles = [90.0] * NUM_SERVOS
        self._lock = threading.Lock()
        self._play_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        if os.path.exists(config_path):
            self.load(config_path)

    def load(self, path: str) -> None:
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        self._expressions = data.get('expressions', {})

    @property
    def expression_names(self):
        return list(self._expressions.keys())

    def play(self, name: str, duration_ms: float = 500.0,
             interp: str = 'linear', blocking: bool = False) -> bool:
        """
        播放表情。
        name: 表情名称
        duration_ms: 过渡时长(ms)
        interp: 'linear' 或 'cubic'
        blocking: True则等待播放完成
        """
        if name not in self._expressions:
            return False

        expr = self._expressions[name]
        target = list(self._current_angles)  # 从当前状态出发

        for servo_id_str, angle in expr.get('servos', {}).items():
            idx = int(servo_id_str) - 1
            if 0 <= idx < NUM_SERVOS:
                target[idx] = float(angle)

        override_dur = expr.get('duration_ms', None)
        if override_dur is not None:
            duration_ms = float(override_dur)

        self._stop_event.set()
        if self._play_thread and self._play_thread.is_alive():
            self._play_thread.join(timeout=0.2)

        self._stop_event.clear()
        self._play_thread = threading.Thread(
            target=self._interpolate,
            args=(list(self._current_angles), target, duration_ms, interp),
            daemon=True
        )
        self._play_thread.start()

        if blocking:
            self._play_thread.join()

        return True

    def stop(self) -> None:
        """中断当前播放"""
        self._stop_event.set()

    def set_angles_direct(self, angles: list) -> None:
        """直接设置角度（跳过插值）"""
        with self._lock:
            self._current_angles = list(angles)
        if self._on_update:
            self._on_update(list(angles))

    # ------------------------------------------------------------------
    def _interpolate(self, start: list, end: list,
                     duration_ms: float, interp: str) -> None:
        dt = 0.02   # 50Hz
        steps = max(1, int(duration_ms / 1000.0 / dt))

        for i in range(steps + 1):
            if self._stop_event.is_set():
                break
            t = i / steps
            if interp == 'cubic':
                t = t * t * (3 - 2 * t)    # smoothstep

            angles = [
                start[j] + (end[j] - start[j]) * t
                for j in range(NUM_SERVOS)
            ]
            with self._lock:
                self._current_angles = angles
            if self._on_update:
                self._on_update(list(angles))

            if i < steps:
                time.sleep(dt)

        # 确保最终到达目标
        if not self._stop_event.is_set():
            with self._lock:
                self._current_angles = list(end)
            if self._on_update:
                self._on_update(list(end))

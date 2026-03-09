"""
舵机参数加载与软限位
从 servos.yaml 读取每路舵机的 min/max/neutral，
对输入角度做 clamp，并管理"锁定"状态（如 id16 的物理限制）。

注意：direction 参数目前标注不准确，本模块已加载但暂不应用方向换算，
      待方向校验完成后在 apply() 中启用。
"""

import yaml
import logging
from typing import Dict, Optional

logger = logging.getLogger(__name__)

NUM_SERVOS = 32


class ServoParam:
    """单路舵机参数"""
    __slots__ = ('id', 'joint', 'neutral', 'min', 'max', 'direction',
                 'note', 'locked_by')

    def __init__(self, data: dict):
        self.id:        int   = int(data['id'])
        self.joint:     str   = str(data.get('joint', f'servo_{self.id}'))
        self.neutral:   float = float(data.get('neutral', 90.0))
        self.min:       float = float(data.get('min', 0.0))
        self.max:       float = float(data.get('max', 180.0))
        self.direction: int   = int(data.get('direction', 1))
        self.note:      str   = str(data.get('note', ''))
        self.locked_by: list  = list(data.get('locked_by', []))

    def clamp(self, angle: float) -> float:
        """将角度限制在 [min, max] 范围内"""
        return max(self.min, min(self.max, angle))

    def is_locked(self, angle: float) -> bool:
        """min == max 时视为锁定（不可移动）"""
        return abs(self.max - self.min) < 0.1

    def __repr__(self):
        return (f'ServoParam(id={self.id}, joint={self.joint}, '
                f'neutral={self.neutral}, range=[{self.min},{self.max}])')


class ServoConfig:
    """
    加载并管理所有32路舵机参数。
    未在 yaml 中定义的舵机使用默认值（neutral=90, min=0, max=180）。
    """

    def __init__(self, yaml_path: Optional[str] = None):
        # 默认配置：全部 0-180°，中位90°
        self._params: Dict[int, ServoParam] = {}
        for i in range(1, NUM_SERVOS + 1):
            self._params[i] = ServoParam({'id': i, 'joint': f'servo_{i}'})

        if yaml_path:
            self.load(yaml_path)

    def load(self, path: str) -> None:
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            servos_list = data.get('servos', []) or []
            for item in servos_list:
                p = ServoParam(item)
                self._params[p.id] = p
            logger.info(f'已加载舵机配置: {path}，共 {len(servos_list)} 路')
        except Exception as e:
            logger.error(f'加载 servos.yaml 失败: {e}，使用默认参数')

    def get(self, servo_id: int) -> ServoParam:
        return self._params.get(servo_id, ServoParam({'id': servo_id}))

    def neutral_angles(self) -> list:
        """返回所有32路的 neutral 角度列表（索引0对应id1）"""
        return [self._params[i].neutral for i in range(1, NUM_SERVOS + 1)]

    def apply(self, angles: list) -> list:
        """
        对输入的32路角度做软限位 clamp。
        同时处理 locked_by 依赖（id16 受 id15/17 约束）。
        direction 换算暂未启用（待校验后开启）。

        angles: 长度32的列表，索引0 = id1
        返回: clamp 后的新列表
        """
        if len(angles) != NUM_SERVOS:
            raise ValueError(f'angles 长度应为32，实际为 {len(angles)}')

        result = list(angles)

        # --- 第一遍：基础 clamp ---
        for i in range(NUM_SERVOS):
            sid = i + 1
            p = self._params[sid]
            result[i] = p.clamp(result[i])

        # --- 第二遍：locked_by 约束（id16受id15/17限制）---
        for i in range(NUM_SERVOS):
            sid = i + 1
            p = self._params[sid]
            if not p.locked_by:
                continue
            # 检查所有依赖舵机是否均到达上限
            all_at_max = all(
                result[dep_id - 1] >= self._params[dep_id].max - 0.5
                for dep_id in p.locked_by
                if dep_id in self._params
            )
            if all_at_max:
                result[i] = p.neutral    # 强制回中位，禁止移动
                logger.debug(
                    f'id{sid}({p.joint}) 因依赖锁定 {p.locked_by} 触发，强制回中位'
                )

        return result

    def summary(self) -> str:
        lines = ['舵机配置摘要:']
        for i in range(1, NUM_SERVOS + 1):
            p = self._params[i]
            locked = '🔒' if p.is_locked(p.neutral) else ''
            lines.append(
                f'  [{i:2d}] {p.joint:<28} neutral={p.neutral:5.1f}° '
                f'range=[{p.min:5.1f},{p.max:5.1f}] dir={p.direction:+d} {locked}'
            )
        return '\n'.join(lines)

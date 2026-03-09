"""
POM SC32 串口通信协议封装
波特率 460800, 8N1
支持 REG / HEX 两种指令模式
"""

import serial
import struct
import time
import threading
import logging

logger = logging.getLogger(__name__)

NUM_SERVOS = 32

# REG 寄存器地址
REG_RESET       = 0x00
REG_SERVO_BASE  = 0x01   # 舵机1: 0x01 ... 舵机32: 0x20
REG_RGB_R       = 0x21
REG_RGB_G       = 0x22
REG_RGB_B       = 0x23
REG_LED         = 0x24
REG_GROUP_BASE  = 0x25   # 组1: 0x25 ... 组6: 0x2A
FRAME_HEAD      = 0xAA


def angle_to_duty(angle: float) -> int:
    """将角度(0-180°)转换为占空比值(0-1000)。标准舵机: 0°→25, 90°→75, 180°→125"""
    angle = max(0.0, min(180.0, float(angle)))
    return round(angle * 5 / 9 + 25)


def duty_to_angle(duty: int) -> float:
    """将占空比值(0-1000)转换为角度(0-180°)"""
    duty = max(0, min(1000, int(duty)))
    return (duty - 25) * 9 / 5


class PomSC32:
    """POM SC32 32路舵机驱动板通信类"""

    def __init__(self, port: str, baudrate: int = 460800, timeout: float = 0.1):
        self._lock = threading.Lock()
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        logger.info(f"串口已打开: {port} @ {baudrate}")

    # ------------------------------------------------------------------
    # REG 指令
    # ------------------------------------------------------------------
    def reg_cmd(self, addr: int, value: int) -> None:
        """发送单条 REG 寄存器指令（6字节）"""
        buf = bytearray(6)
        buf[0] = FRAME_HEAD
        buf[1] = 0x00
        buf[2] = addr & 0xFF
        buf[3] = (value >> 8) & 0xFF
        buf[4] = value & 0xFF
        buf[5] = (buf[0] + buf[1] + buf[2] + buf[3] + buf[4]) & 0xFF
        with self._lock:
            self.ser.write(buf)

    def set_group_freq(self, freq: int = 50) -> None:
        """设置全部6组 PWM 频率（Hz），标准舵机用50Hz"""
        freq = max(50, min(15000, freq))
        for addr in range(REG_GROUP_BASE, REG_GROUP_BASE + 6):
            self.reg_cmd(addr, freq)
        logger.info(f"6组PWM频率已设置为 {freq} Hz")

    def set_servo_reg(self, servo_id: int, angle: float) -> None:
        """用 REG 指令控制单个舵机（servo_id: 1-32, angle: 0-180°）"""
        if not 1 <= servo_id <= NUM_SERVOS:
            raise ValueError(f"servo_id 超范围: {servo_id}")
        duty = angle_to_duty(angle)
        self.reg_cmd(REG_SERVO_BASE + servo_id - 1, duty)

    # ------------------------------------------------------------------
    # HEX 指令（推荐：一次下发全部32路）
    # ------------------------------------------------------------------
    def hex_cmd(self, duties: list) -> bool:
        """
        发送 HEX 批量指令（65字节），控制全部32路舵机。
        duties: 长度32的列表，每个元素为占空比值(0-1000)。
        返回 True 表示驱动板确认成功。
        """
        if len(duties) != NUM_SERVOS:
            raise ValueError(f"duties 长度必须为32，实际为 {len(duties)}")

        buf = bytearray(65)
        checksum = 0
        for i, d in enumerate(duties):
            d = max(0, min(1000, int(d)))
            buf[i * 2]     = (d >> 8) & 0xFF
            buf[i * 2 + 1] = d & 0xFF
            checksum += buf[i * 2] + buf[i * 2 + 1]
        buf[64] = checksum & 0xFF

        with self._lock:
            self.ser.write(buf)
            resp = self.ser.read(6)   # 期望 b'OK_HEX'

        ok = (resp == b'OK_HEX')
        if not ok:
            logger.debug(f"HEX指令响应异常: {resp!r}")
        return ok

    def set_angles(self, angles: list) -> bool:
        """
        控制32路舵机角度（最常用接口）。
        angles: 长度32的列表，每个元素为角度(0-180°)。
        """
        duties = [angle_to_duty(a) for a in angles]
        return self.hex_cmd(duties)

    # ------------------------------------------------------------------
    # 初始化 / 工具
    # ------------------------------------------------------------------
    def initialize(self, freq: int = 50) -> None:
        """初始化驱动板：设置PWM频率"""
        self.set_group_freq(freq)
        time.sleep(0.05)

    def go_neutral(self) -> bool:
        """所有舵机回中位(90°)"""
        return self.set_angles([90.0] * NUM_SERVOS)

    def reset(self) -> None:
        """系统复位"""
        self.reg_cmd(REG_RESET, 1)

    def close(self) -> None:
        """关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("串口已关闭")

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

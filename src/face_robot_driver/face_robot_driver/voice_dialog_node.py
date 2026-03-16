"""
语音对话节点

通过豆包 (Doubao) Realtime Dialog API 实现语音/文字对话。

流程:
  audio 模式: 麦克风采集 PCM → WebSocket → 服务端 ASR/LLM/TTS → 扬声器
  text  模式: 订阅文字话题 → ChatTextQuery → 服务端 LLM/TTS → 扬声器

话题（发布）:
  /face/voice/user_text   String  用户语音识别文本（audio 模式）
  /face/voice/bot_text    String  机器人回复文本
  /face/voice/status      String  连接状态

话题（订阅）:
  /face/voice/text_input  String  文字输入（text 模式）

参数:
  app_id            str    豆包 APP ID
  access_key        str    豆包 Access Key
  speaker           str    TTS 音色（默认 zh_male_yunzhou_jupiter_bigtts）
  system_role       str    系统角色设定
  bot_name          str    机器人名称
  input_mode        str    输入模式: 'audio'=麦克风, 'text'=文字（默认 audio）
  sample_rate_in    int    麦克风采样率（默认 16000）
  sample_rate_out   int    播放采样率（默认 24000）
  chunk_size        int    每次采集帧数（默认 3200，即 200ms@16kHz）
"""

import asyncio
import queue
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoiceDialogNode(Node):

    def __init__(self):
        super().__init__('voice_dialog')

        # --- 参数 ---
        self.declare_parameter('app_id', '6496508405')
        self.declare_parameter('access_key', 'iU76WCb5fJU1w_mmdiTnyAZOqlJVJLpF')
        self.declare_parameter('speaker', 'zh_male_yunzhou_jupiter_bigtts')
        self.declare_parameter('system_role',
                               '你是一个友好的机器人助手，回答简洁明了，每次回复控制在两三句话以内。')
        self.declare_parameter('bot_name', '')
        self.declare_parameter('input_mode', 'audio')
        self.declare_parameter('sample_rate_in', 16000)
        self.declare_parameter('sample_rate_out', 24000)
        self.declare_parameter('chunk_size', 3200)

        self._app_id = self.get_parameter('app_id').value
        self._access_key = self.get_parameter('access_key').value
        self._speaker = self.get_parameter('speaker').value
        self._system_role = self.get_parameter('system_role').value
        self._bot_name = self.get_parameter('bot_name').value
        self._input_mode = self.get_parameter('input_mode').value
        self._sr_in = self.get_parameter('sample_rate_in').value
        self._sr_out = self.get_parameter('sample_rate_out').value
        self._chunk = self.get_parameter('chunk_size').value

        # --- 发布 ---
        self._pub_user = self.create_publisher(String, '/face/voice/user_text', 10)
        self._pub_bot = self.create_publisher(String, '/face/voice/bot_text', 10)
        self._pub_status = self.create_publisher(String, '/face/voice/status', 10)

        # --- 文字输入订阅 ---
        self._text_queue = queue.Queue(maxsize=50)
        self._sub_text = self.create_subscription(
            String, '/face/voice/text_input', self._text_input_cb, 10)

        # --- 音频播放队列 ---
        self._audio_queue = queue.Queue(maxsize=200)
        self._playing = True
        self._user_speaking = False

        # --- aplay 子进程播放 (避免 pyaudio write 兼容性问题) ---
        self._aplay_proc = subprocess.Popen(
            ['aplay', '-r', str(self._sr_out), '-f', 'S16_LE',
             '-c', '1', '-q', '-'],
            stdin=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )

        # --- 播放线程 ---
        self._player_thread = threading.Thread(
            target=self._audio_player_loop, daemon=True, name='audio_player')
        self._player_thread.start()

        # --- asyncio 线程（WebSocket + 输入） ---
        self._stop_event = threading.Event()
        self._async_thread = threading.Thread(
            target=self._run_async, daemon=True, name='doubao_async')
        self._async_thread.start()

        self.get_logger().info(
            f'语音对话节点已启动  mode={self._input_mode}  '
            f'app_id={self._app_id}  speaker={self._speaker}')

    # ------------------------------------------------------------------
    # 文字输入回调（ROS 主线程）
    # ------------------------------------------------------------------

    def _text_input_cb(self, msg: String):
        """收到文字输入话题，放入队列由 asyncio 线程发送。"""
        text = msg.data.strip()
        if text:
            self.get_logger().info(f'文字输入: {text}')
            try:
                self._text_queue.put_nowait(text)
            except queue.Full:
                self.get_logger().warn('文字输入队列已满，丢弃')

    # ------------------------------------------------------------------
    # 音频播放线程
    # ------------------------------------------------------------------

    def _audio_player_loop(self):
        """从队列取出 PCM 数据通过 aplay 写入扬声器。"""
        while self._playing:
            try:
                data = self._audio_queue.get(timeout=0.5)
                if data is not None and self._aplay_proc.stdin:
                    self._aplay_proc.stdin.write(data)
                    self._aplay_proc.stdin.flush()
            except queue.Empty:
                pass
            except (BrokenPipeError, OSError):
                break
            except Exception as e:
                self.get_logger().warn(f'播放错误: {e}')
                time.sleep(0.1)

    # ------------------------------------------------------------------
    # asyncio 线程
    # ------------------------------------------------------------------

    def _run_async(self):
        """在独立线程中运行 asyncio 事件循环。"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._async_main())
        except Exception as e:
            self.get_logger().error(f'async 线程异常: {e}')
        finally:
            loop.close()

    async def _async_main(self):
        """连接豆包、启动输入采集和接收循环。"""
        from .doubao_client import DoubaoClient

        client = DoubaoClient(
            app_id=self._app_id,
            access_key=self._access_key,
            speaker=self._speaker,
            system_role=self._system_role,
            bot_name=self._bot_name,
            input_mode=self._input_mode,
            on_audio=self._on_audio,
            on_user_text=self._on_user_text,
            on_bot_text=self._on_bot_text,
            on_status=self._on_status,
        )

        # 连接（带重试）
        for attempt in range(3):
            ok = await client.connect_and_start()
            if ok:
                break
            self.get_logger().warn(
                f'连接失败, 第 {attempt+1}/3 次, 5秒后重试...')
            await asyncio.sleep(5)
        else:
            self.get_logger().error('无法连接豆包服务，节点退出')
            return

        # 根据模式启动输入任务
        tasks = [asyncio.ensure_future(client.receive_loop())]

        if self._input_mode == 'audio':
            tasks.append(asyncio.ensure_future(self._mic_loop(client)))
        else:
            self.get_logger().info(
                '文字模式已启动，通过 /face/voice/text_input 话题发送文字')

        # 文字输入轮询（两种模式下都可用）
        tasks.append(asyncio.ensure_future(self._text_poll_loop(client)))

        # 等待任一结束
        done, pending = await asyncio.wait(
            tasks, return_when=asyncio.FIRST_COMPLETED)
        for t in pending:
            t.cancel()

        await client.close()
        self.get_logger().info('豆包对话已断开')

    async def _mic_loop(self, client):
        """采集麦克风音频（通过 arecord），发送到豆包。"""
        # chunk_bytes = chunk_size * 2 (int16 = 2 bytes per sample)
        chunk_bytes = self._chunk * 2

        try:
            arecord = subprocess.Popen(
                ['arecord', '-r', str(self._sr_in), '-f', 'S16_LE',
                 '-c', '1', '-q', '-'],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            self.get_logger().error(f'无法打开麦克风(arecord): {e}')
            return

        self.get_logger().info('麦克风已打开，请讲话...')

        try:
            while not self._stop_event.is_set() and client.running:
                if not client.input_ready:
                    await asyncio.sleep(0.05)
                    continue
                try:
                    audio_data = arecord.stdout.read(chunk_bytes)
                    if not audio_data:
                        break
                    await client.send_audio(audio_data)
                    await asyncio.sleep(0.01)
                except Exception as e:
                    self.get_logger().warn(f'麦克风读取错误: {e}')
                    await asyncio.sleep(0.1)
        finally:
            arecord.terminate()
            arecord.wait()

    async def _text_poll_loop(self, client):
        """轮询文字输入队列，发送 ChatTextQuery 到豆包。"""
        while not self._stop_event.is_set() and client.running:
            if not client.input_ready:
                await asyncio.sleep(0.05)
                continue
            try:
                text = self._text_queue.get_nowait()
                await client.chat_text_query(text)
                # 发布到 user_text 话题
                msg = String()
                msg.data = text
                self._pub_user.publish(msg)
            except queue.Empty:
                pass
            await asyncio.sleep(0.1)

    # ------------------------------------------------------------------
    # 回调（从 asyncio 线程调用）
    # ------------------------------------------------------------------

    def _on_audio(self, pcm_data: bytes):
        """收到 TTS 音频数据。"""
        if not self._user_speaking:
            try:
                self._audio_queue.put_nowait(pcm_data)
            except queue.Full:
                pass  # 丢弃溢出数据

    def _on_user_text(self, text: str, is_final: bool):
        """收到 ASR 识别文本。"""
        if is_final:
            self.get_logger().info(f'用户: {text}')
        msg = String()
        msg.data = text
        self._pub_user.publish(msg)

    def _on_bot_text(self, text: str):
        """收到机器人回复文本。"""
        self.get_logger().info(f'机器人: {text}')
        msg = String()
        msg.data = text
        self._pub_bot.publish(msg)

    def _on_status(self, status: str):
        """连接状态变更。"""
        self.get_logger().info(f'状态: {status}')
        msg = String()
        msg.data = status
        self._pub_status.publish(msg)

        if status == 'user_speaking':
            self._user_speaking = True
            # 清空播放队列（用户打断）
            while not self._audio_queue.empty():
                try:
                    self._audio_queue.get_nowait()
                except queue.Empty:
                    break
        elif status == 'tts_ended':
            self._user_speaking = False

    # ------------------------------------------------------------------
    # 销毁
    # ------------------------------------------------------------------

    def destroy_node(self):
        self._stop_event.set()
        self._playing = False
        try:
            if self._aplay_proc.stdin:
                self._aplay_proc.stdin.close()
            self._aplay_proc.terminate()
            self._aplay_proc.wait(timeout=2)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceDialogNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
Doubao Realtime Dialog WebSocket client.

Manages WebSocket connection, session lifecycle, audio streaming.
Adapted from official example for ROS 2 integration.
"""

import asyncio
import gzip
import json
import logging
import uuid
from typing import Optional, Callable

import websockets

from . import doubao_protocol as proto

logger = logging.getLogger(__name__)

WS_URL = 'wss://openspeech.bytedance.com/api/v3/realtime/dialogue'


class DoubaoClient:
    """Async WebSocket client for Doubao Realtime Dialog API."""

    def __init__(self,
                 app_id: str,
                 access_key: str,
                 speaker: str = 'zh_male_yunzhou_jupiter_bigtts',
                 system_role: str = '',
                 bot_name: str = '',
                 input_mode: str = 'audio',
                 on_audio: Optional[Callable] = None,
                 on_user_text: Optional[Callable] = None,
                 on_bot_text: Optional[Callable] = None,
                 on_status: Optional[Callable] = None):
        self._app_id = app_id
        self._access_key = access_key
        self._speaker = speaker
        self._system_role = system_role
        self._bot_name = bot_name
        self._input_mode = input_mode  # 'audio', 'text'

        # callbacks (called from asyncio thread)
        self._on_audio = on_audio
        self._on_user_text = on_user_text
        self._on_bot_text = on_bot_text
        self._on_status = on_status

        self._ws = None
        self._session_id = str(uuid.uuid4())
        self._connect_id = str(uuid.uuid4())
        self._running = False
        self._is_user_speaking = False
        self._input_ready = False

    @property
    def running(self) -> bool:
        return self._running

    @property
    def input_ready(self) -> bool:
        return self._input_ready

    async def connect_and_start(self):
        """Connect to WebSocket, start connection and session."""
        headers = {
            'X-Api-App-ID': self._app_id,
            'X-Api-Access-Key': self._access_key,
            'X-Api-Resource-Id': 'volc.speech.dialog',
            'X-Api-App-Key': 'PlgvMymc7f3tQnJ6',
            'X-Api-Connect-Id': self._connect_id,
        }

        self._notify_status('connecting')
        try:
            self._ws = await websockets.connect(
                WS_URL,
                extra_headers=headers,
                ping_interval=None,
            )
        except Exception as e:
            logger.error(f'WebSocket connect failed: {e}')
            self._notify_status(f'connect_failed: {e}')
            return False

        logid = self._ws.response_headers.get('X-Tt-Logid', '')
        logger.info(f'Connected, logid={logid}')

        # --- StartConnection (event=1) ---
        buf = bytearray(proto.generate_header())
        buf.extend(proto.START_CONNECTION.to_bytes(4, 'big'))
        payload = gzip.compress(b'{}')
        buf.extend(len(payload).to_bytes(4, 'big'))
        buf.extend(payload)
        await self._ws.send(buf)
        resp = await self._ws.recv()
        r = proto.parse_response(resp)
        logger.info(f'StartConnection: {r}')

        # --- StartSession (event=100) ---
        session_cfg = {
            'asr': {
                'extra': {
                    'end_smooth_window_ms': 1500,
                },
            },
            'tts': {
                'speaker': self._speaker,
                'audio_config': {
                    'channel': 1,
                    'format': 'pcm_s16le',
                    'sample_rate': 24000,
                },
            },
            'dialog': {
                'bot_name': self._bot_name or '',
                'system_role': self._system_role or '',
                'location': {'city': ''},
                'extra': {
                    'strict_audit': False,
                    'recv_timeout': 30,
                    'input_mod': self._input_mode,
                    'model': '1.2.1.1',
                },
            },
        }
        buf = proto.pack_json_event(
            proto.START_SESSION, self._session_id, session_cfg)
        await self._ws.send(buf)
        resp = await self._ws.recv()
        r = proto.parse_response(resp)
        logger.info(f'StartSession: {r}')

        self._running = True
        self._input_ready = False
        self._notify_status('connected')

        if self._input_mode == 'text':
            self._input_ready = True
            self._notify_status('input_ready')
            return True

        # --- SayHello (event=300) ---
        hello_payload = {'content': ''}
        buf = proto.pack_json_event(
            proto.SAY_HELLO, self._session_id, hello_payload)
        await self._ws.send(buf)
        return True

    async def send_audio(self, pcm_data: bytes):
        """Send a chunk of PCM audio to the server."""
        if not self._ws or not self._running:
            return
        buf = proto.pack_audio(proto.TASK_REQUEST, self._session_id, pcm_data)
        try:
            await self._ws.send(buf)
        except Exception as e:
            logger.warning(f'send_audio error: {e}')

    async def chat_text_query(self, content: str):
        """Send a text query to the model (ChatTextQuery, event=501)."""
        if not self._ws or not self._running:
            return
        payload = {'content': content}
        buf = proto.pack_json_event(
            proto.CHAT_TEXT_QUERY, self._session_id, payload)
        try:
            await self._ws.send(buf)
            logger.info(f'ChatTextQuery sent: {content}')
        except Exception as e:
            logger.warning(f'chat_text_query error: {e}')

    async def receive_loop(self):
        """Receive and dispatch server messages. Runs until session ends."""
        try:
            while self._running:
                resp = await self._ws.recv()
                data = proto.parse_response(resp)
                if not data:
                    continue
                self._handle_response(data)

                event = data.get('event', 0)
                if event in (proto.SESSION_FINISHED, proto.SESSION_FAILED):
                    logger.info(f'Session ended, event={event}')
                    break
        except asyncio.CancelledError:
            pass
        except websockets.exceptions.ConnectionClosed as e:
            logger.warning(f'WebSocket closed: {e}')
        except Exception as e:
            logger.error(f'receive_loop error: {e}')
        finally:
            self._running = False
            self._notify_status('disconnected')

    def _handle_response(self, data: dict):
        msg_type = data.get('message_type', '')
        event = data.get('event', 0)
        payload = data.get('payload_msg')

        # TTS audio data
        if msg_type == 'SERVER_ACK' and isinstance(payload, bytes):
            if self._on_audio and not self._is_user_speaking:
                self._on_audio(payload)
            return

        if msg_type == 'SERVER_ERROR':
            logger.error(f'Server error: {payload}')
            self._notify_status(f'error: {payload}')
            return

        # ASR info (user starts speaking) -> clear audio queue
        if event == proto.ASR_INFO:
            self._is_user_speaking = True
            self._notify_status('user_speaking')
            return

        # ASR response (partial/final text)
        # SDK format: {"results": [{"text": ..., "is_interim": ...}]}
        if event == proto.ASR_RESPONSE:
            if isinstance(payload, dict):
                results = payload.get('results', [])
                if results and isinstance(results, list):
                    item = results[0]
                    text = item.get('text', '')
                    is_final = not item.get('is_interim', True)
                    if text and self._on_user_text:
                        self._on_user_text(text, is_final)
                else:
                    text = payload.get('text', '')
                    if text and self._on_user_text:
                        self._on_user_text(text, False)

        # ASR ended (user stopped speaking)
        if event == proto.ASR_ENDED:
            self._is_user_speaking = False

        # Chat response (bot text) - field is 'content' per SDK doc
        if event == proto.CHAT_RESPONSE:
            if isinstance(payload, dict):
                text = payload.get('content', '') or payload.get('text', '')
                if text and self._on_bot_text:
                    self._on_bot_text(text)

        # TTS ended
        if event == proto.TTS_ENDED:
            if not self._input_ready:
                self._input_ready = True
                self._notify_status('input_ready')
            self._notify_status('tts_ended')

    async def close(self):
        """Gracefully close session and connection."""
        if not self._ws:
            return
        self._running = False
        self._input_ready = False
        try:
            # FinishSession
            buf = proto.pack_json_event(
                proto.FINISH_SESSION, self._session_id, {})
            await self._ws.send(buf)

            # FinishConnection
            fc_buf = bytearray(proto.generate_header())
            fc_buf.extend(proto.FINISH_CONNECTION.to_bytes(4, 'big'))
            payload = gzip.compress(b'{}')
            fc_buf.extend(len(payload).to_bytes(4, 'big'))
            fc_buf.extend(payload)
            await self._ws.send(fc_buf)
            await asyncio.sleep(0.2)
            await self._ws.close()
        except Exception as e:
            logger.warning(f'close error: {e}')
        self._notify_status('closed')

    def _notify_status(self, status: str):
        if self._on_status:
            self._on_status(status)

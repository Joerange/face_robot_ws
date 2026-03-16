"""
Doubao Realtime Dialog WebSocket binary protocol.

Adapted from official example: /home/tuf/Downloads/python3.7/protocol.py
"""

import gzip
import json

PROTOCOL_VERSION = 0b0001
DEFAULT_HEADER_SIZE = 0b0001

# Message Type
CLIENT_FULL_REQUEST = 0b0001
CLIENT_AUDIO_ONLY_REQUEST = 0b0010
SERVER_FULL_RESPONSE = 0b1001
SERVER_ACK = 0b1011
SERVER_ERROR_RESPONSE = 0b1111

# Message Type Specific Flags
NO_SEQUENCE = 0b0000
POS_SEQUENCE = 0b0001
NEG_SEQUENCE = 0b0010
NEG_SEQUENCE_1 = 0b0011
MSG_WITH_EVENT = 0b0100

# Serialization
NO_SERIALIZATION = 0b0000
JSON_SERIAL = 0b0001

# Compression
NO_COMPRESSION = 0b0000
GZIP_COMPRESS = 0b0001

# --- Server Events ---
SESSION_STARTED = 150
SESSION_FINISHED = 152
SESSION_FAILED = 153
HELLO_RESPONSE = 300
TTS_STARTED = 350
TTS_RESPONSE = 352       # audio data
TTS_ENDED = 359
ASR_INFO = 450            # user start speaking (clear audio buffer)
ASR_RESPONSE = 451        # partial/final ASR text
ASR_ENDED = 459           # user finished speaking
CHAT_RESPONSE = 550       # bot text response

# --- Client Events ---
START_CONNECTION = 1
FINISH_CONNECTION = 2
START_SESSION = 100
FINISH_SESSION = 102
TASK_REQUEST = 200        # audio data
SAY_HELLO = 300
CHAT_TTS_TEXT = 500
CHAT_TEXT_QUERY = 501


def generate_header(
    message_type=CLIENT_FULL_REQUEST,
    message_type_specific_flags=MSG_WITH_EVENT,
    serial_method=JSON_SERIAL,
    compression_type=GZIP_COMPRESS,
):
    header = bytearray()
    header_size = 1
    header.append((PROTOCOL_VERSION << 4) | header_size)
    header.append((message_type << 4) | message_type_specific_flags)
    header.append((serial_method << 4) | compression_type)
    header.append(0x00)
    return header


def pack_json_event(event_id: int, session_id: str, payload: dict) -> bytearray:
    """Pack a JSON event with session_id."""
    buf = bytearray(generate_header())
    buf.extend(event_id.to_bytes(4, 'big'))
    sid_bytes = session_id.encode()
    buf.extend(len(sid_bytes).to_bytes(4, 'big'))
    buf.extend(sid_bytes)
    payload_bytes = gzip.compress(json.dumps(payload).encode())
    buf.extend(len(payload_bytes).to_bytes(4, 'big'))
    buf.extend(payload_bytes)
    return buf


def pack_audio(event_id: int, session_id: str, audio: bytes) -> bytearray:
    """Pack an audio-only event."""
    buf = bytearray(generate_header(
        message_type=CLIENT_AUDIO_ONLY_REQUEST,
        serial_method=NO_SERIALIZATION,
    ))
    buf.extend(event_id.to_bytes(4, 'big'))
    sid_bytes = session_id.encode()
    buf.extend(len(sid_bytes).to_bytes(4, 'big'))
    buf.extend(sid_bytes)
    payload_bytes = gzip.compress(audio)
    buf.extend(len(payload_bytes).to_bytes(4, 'big'))
    buf.extend(payload_bytes)
    return buf


def parse_response(data) -> dict:
    """Parse a server response binary frame."""
    if isinstance(data, str):
        return {}

    message_type = data[1] >> 4
    flags = data[1] & 0x0f
    serial_method = data[2] >> 4
    compression = data[2] & 0x0f
    header_size = data[0] & 0x0f
    payload = data[header_size * 4:]

    result = {}
    start = 0

    if message_type in (SERVER_FULL_RESPONSE, SERVER_ACK):
        result['message_type'] = ('SERVER_FULL_RESPONSE'
                                  if message_type == SERVER_FULL_RESPONSE
                                  else 'SERVER_ACK')
        if flags & NEG_SEQUENCE:
            result['seq'] = int.from_bytes(payload[:4], 'big', signed=False)
            start += 4
        if flags & MSG_WITH_EVENT:
            result['event'] = int.from_bytes(payload[start:start+4], 'big',
                                             signed=False)
            start += 4

        payload = payload[start:]
        sid_len = int.from_bytes(payload[:4], 'big', signed=True)
        result['session_id'] = payload[4:4+sid_len].decode(errors='replace')
        payload = payload[4 + sid_len:]
        payload_size = int.from_bytes(payload[:4], 'big', signed=False)
        payload_msg = payload[4:]

    elif message_type == SERVER_ERROR_RESPONSE:
        result['message_type'] = 'SERVER_ERROR'
        result['code'] = int.from_bytes(payload[:4], 'big', signed=False)
        payload_size = int.from_bytes(payload[4:8], 'big', signed=False)
        payload_msg = payload[8:]
    else:
        return result

    if payload_msg is None:
        return result

    if compression == GZIP_COMPRESS:
        payload_msg = gzip.decompress(payload_msg)
    if serial_method == JSON_SERIAL:
        payload_msg = json.loads(payload_msg.decode())
    elif serial_method != NO_SERIALIZATION:
        payload_msg = payload_msg.decode(errors='replace')

    result['payload_msg'] = payload_msg
    result['payload_size'] = payload_size
    return result

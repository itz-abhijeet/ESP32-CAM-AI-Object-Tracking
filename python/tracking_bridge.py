"""
tracking_bridge.py
──────────────────
Lightweight, zero-blocking bridge between the AI tracking scripts and the
AstraNex UI server (ui_server.py).

Usage in ai_tracking.py / test_webcam_tracking.py:

    from tracking_bridge import push_state, push_frame

Call push_state(...) each loop iteration with the latest tracking state.
Call push_frame(jpeg_bytes) each loop iteration with the latest JPEG frame.

Both calls are fire-and-forget UDP sends to localhost — they never block
the main tracking loop.
"""

import socket
import json
import struct
import threading

_SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_STATE_ADDR = ("127.0.0.1", 9001)   # ui_server.py listens for state JSON here
_FRAME_ADDR = ("127.0.0.1", 9002)   # ui_server.py listens for JPEG frames here

# Maximum UDP payload. Frames larger than this are split into chunks.
_MAX_CHUNK = 60000

_frame_seq = 0
_frame_lock = threading.Lock()


def push_state(
    status: str,
    cls_name: str = "",
    confidence: float = 0.0,
    pan_delta: int = 0,
    tilt_delta: int = 0,
    error_x: int = 0,
    error_y: int = 0,
    smooth_x: int | None = None,
    smooth_y: int | None = None,
    bbox: tuple | None = None,   # (x1, y1, x2, y2)
    fps: float = 0.0,
    frame_w: int = 0,
    frame_h: int = 0,
):
    """Send a tracking-state snapshot to the UI server (non-blocking UDP)."""
    payload = {
        "type": "state",
        "status": status,
        "cls_name": cls_name,
        "confidence": round(confidence, 4),
        "pan_delta": pan_delta,
        "tilt_delta": tilt_delta,
        "error_x": error_x,
        "error_y": error_y,
        "smooth_x": smooth_x,
        "smooth_y": smooth_y,
        "bbox": list(bbox) if bbox else None,
        "fps": round(fps, 1),
        "frame_w": frame_w,
        "frame_h": frame_h,
    }
    try:
        data = json.dumps(payload).encode()
        _SOCK.sendto(data, _STATE_ADDR)
    except Exception:
        pass


def push_frame(jpeg_bytes: bytes):
    """Send a JPEG frame to the UI server as chunked UDP packets (non-blocking)."""
    global _frame_seq
    if not jpeg_bytes:
        return
    try:
        with _frame_lock:
            seq = _frame_seq
            _frame_seq = (_frame_seq + 1) % 65536

        total = len(jpeg_bytes)
        num_chunks = (total + _MAX_CHUNK - 1) // _MAX_CHUNK

        for i in range(num_chunks):
            chunk = jpeg_bytes[i * _MAX_CHUNK:(i + 1) * _MAX_CHUNK]
            # Header: seq(2) | chunk_idx(2) | num_chunks(2)
            header = struct.pack(">HHH", seq, i, num_chunks)
            _SOCK.sendto(header + chunk, _FRAME_ADDR)
    except Exception:
        pass

"""
ui_server.py
────────────
AstraNex Tracking UI Server

Listens for tracking data pushed by tracking_bridge.py over UDP and
re-broadcasts it to connected browser clients over WebSocket.
Also serves the MJPEG video stream and the dashboard HTML file.

Usage:
    python ui_server.py

Then open http://localhost:8765 in your browser (redirects to the HTML UI).
Or open python/tracking_ui.html directly in the browser.

While this server is running, start the tracker in a separate terminal:
    python ai_tracking.py
    or
    python test_webcam_tracking.py

The dashboard will auto-connect and show live data.

Ports:
  8765  WebSocket  (ws://localhost:8765)
  8766  HTTP       (http://localhost:8766)  → MJPEG + HTML file
  9001  UDP in     state JSON from tracking_bridge
  9002  UDP in     JPEG frames  from tracking_bridge
"""

import asyncio
import json
import socket
import struct
import threading
import time
import os
from pathlib import Path
from http.server import BaseHTTPRequestHandler, HTTPServer

import websockets

# ─── Config ────────────────────────────────────────────────────────────────────
WS_HOST = "localhost"
WS_PORT = 8765
HTTP_PORT = 8766
STATE_UDP_PORT = 9001
FRAME_UDP_PORT = 9002

THIS_DIR = Path(__file__).resolve().parent
HTML_FILE = THIS_DIR / "tracking_ui.html"
LOGO_FILE = THIS_DIR.parent / "AstraNex_Logo.png"

# ─── Shared State ──────────────────────────────────────────────────────────────
_connected_clients: set = set()
_latest_state: dict = {"type": "state", "status": "NO TARGET"}
_latest_jpeg: bytes = b""
_state_lock = threading.Lock()
_frame_lock = threading.Lock()
_frame_buffers: dict[int, dict] = {}   # seq -> {chunks}

# ─── WebSocket Server ──────────────────────────────────────────────────────────
async def ws_handler(websocket):
    _connected_clients.add(websocket)
    print(f"[WS] Client connected. Total: {len(_connected_clients)}")
    try:
        # Send latest state immediately on connect
        with _state_lock:
            state = dict(_latest_state)
        await websocket.send(json.dumps(state))
        # Keep alive — the UDP receiver broadcasts; nothing to await here
        async for _ in websocket:
            pass
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        _connected_clients.discard(websocket)
        print(f"[WS] Client disconnected. Total: {len(_connected_clients)}")


async def broadcast_state(state: dict):
    if not _connected_clients:
        return
    msg = json.dumps(state)
    dead = set()
    for ws in list(_connected_clients):
        try:
            await ws.send(msg)
        except Exception:
            dead.add(ws)
    _connected_clients -= dead


async def broadcast_frame(jpeg: bytes):
    if not _connected_clients or not jpeg:
        return
    # Send as binary WebSocket message
    dead = set()
    for ws in list(_connected_clients):
        try:
            await ws.send(jpeg)
        except Exception:
            dead.add(ws)
    _connected_clients -= dead


# ─── UDP Listeners (run in threads, push to asyncio loop) ─────────────────────
def _udp_state_listener(loop: asyncio.AbstractEventLoop):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
    sock.bind(("127.0.0.1", STATE_UDP_PORT))
    print(f"[UDP] Listening for state on port {STATE_UDP_PORT}")
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            state = json.loads(data.decode())
            with _state_lock:
                global _latest_state
                _latest_state = state
            asyncio.run_coroutine_threadsafe(broadcast_state(state), loop)
        except Exception:
            pass


def _udp_frame_listener(loop: asyncio.AbstractEventLoop):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2 * 1024 * 1024)
    sock.bind(("127.0.0.1", FRAME_UDP_PORT))
    print(f"[UDP] Listening for frames on port {FRAME_UDP_PORT}")
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            if len(data) < 6:
                continue
            seq, chunk_idx, num_chunks = struct.unpack(">HHH", data[:6])
            chunk = data[6:]

            if seq not in _frame_buffers:
                _frame_buffers[seq] = {"total": num_chunks, "chunks": {}}
            _frame_buffers[seq]["chunks"][chunk_idx] = chunk

            if len(_frame_buffers[seq]["chunks"]) == num_chunks:
                # Reassemble full JPEG
                parts = [_frame_buffers.pop(seq)["chunks"][i] for i in range(num_chunks)]
                jpeg = b"".join(parts)
                with _frame_lock:
                    global _latest_jpeg
                    _latest_jpeg = jpeg
                asyncio.run_coroutine_threadsafe(broadcast_frame(jpeg), loop)

            # Prune stale buffers (keep last 10 seqs)
            old = sorted(_frame_buffers.keys())
            for old_seq in old[:-10]:
                _frame_buffers.pop(old_seq, None)
        except Exception:
            pass


# ─── HTTP Server (serves HTML + logo + MJPEG) ─────────────────────────────────
class _Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # suppress noisy logs

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            if HTML_FILE.exists():
                content = HTML_FILE.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
            else:
                self.send_response(404)
                self.end_headers()

        elif self.path == "/logo.png":
            if LOGO_FILE.exists():
                content = LOGO_FILE.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "image/png")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
            else:
                self.send_response(404)
                self.end_headers()

        elif self.path == "/video":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                while True:
                    with _frame_lock:
                        jpeg = _latest_jpeg
                    if jpeg:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                    time.sleep(0.033)  # ~30 fps cap
            except Exception:
                pass
        else:
            self.send_response(404)
            self.end_headers()


def _run_http():
    HTTPServer.allow_reuse_address = True
    server = HTTPServer(("0.0.0.0", HTTP_PORT), _Handler)
    print(f"[HTTP] Serving dashboard at http://localhost:{HTTP_PORT}")
    server.serve_forever()


# ─── Entry Point ───────────────────────────────────────────────────────────────
async def main():
    loop = asyncio.get_event_loop()

    # Start UDP listeners in background threads
    threading.Thread(target=_udp_state_listener, args=(loop,), daemon=True).start()
    threading.Thread(target=_udp_frame_listener, args=(loop,), daemon=True).start()
    threading.Thread(target=_run_http, daemon=True).start()

    print(f"[WS]   WebSocket server at ws://{WS_HOST}:{WS_PORT}")
    print(f"[HTTP] Dashboard at       http://localhost:{HTTP_PORT}")
    print(f"\n>>> Open http://localhost:{HTTP_PORT} in your browser <<<\n")
    print("Now run your tracker (ai_tracking.py / test_webcam_tracking.py) in another terminal.\n")

    async with websockets.serve(ws_handler, WS_HOST, WS_PORT, reuse_port=False):
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    asyncio.run(main())

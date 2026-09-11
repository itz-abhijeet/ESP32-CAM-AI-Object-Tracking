import cv2
import time
import os
from pathlib import Path
from dotenv import load_dotenv
from ultralytics import YOLO

import sys
import threading

# Optional: push live data to AstraNex UI server (ui_server.py)
try:
    from tracking_bridge import push_state, push_frame as _push_frame
    UI_ENABLED = True
except ImportError:
    UI_ENABLED = False
    def push_state(*a, **kw): pass
    def _push_frame(*a, **kw): pass

# Load .env from python directory or project root
load_dotenv(dotenv_path=Path(__file__).resolve().parent / ".env")
load_dotenv(dotenv_path=Path(__file__).resolve().parent.parent / ".env")

# ==========================================
# TEST SETTINGS (NO HARDWARE REQUIRED)
# ==========================================

# Support URL or index from CLI arguments or environment variables:
# e.g., CAMERA_SOURCE=http://10.220.217.17:81/stream or CAMERA_INDEX=0
def get_camera_source():
    # 1. Check CLI arguments (e.g., python test_webcam_tracking.py http://10.220.217.17:81/)
    for arg in sys.argv[1:]:
        if not arg.startswith("-"):
            return arg
        if arg.startswith("--source=") or arg.startswith("--url="):
            return arg.split("=", 1)[1]

    # 2. Check CAMERA_SOURCE or CAMERA_URL or CAMERA_INDEX from environment
    env_source = os.getenv("CAMERA_SOURCE")
    if env_source:
        return env_source

    env_url = os.getenv("CAMERA_URL")
    if env_url and "YOUR_ESP32_CAM_IP" not in env_url:
        return env_url

    return os.getenv("CAMERA_INDEX", "0")

CAMERA_SOURCE_RAW = get_camera_source()

# Determine if source is an integer index or a URL
def normalize_camera_source(source_str):
    source_str = str(source_str).strip()
    if source_str.isdigit():
        return int(source_str)
    # If it is an ESP32-CAM base URL like http://10.220.217.17:81/ or http://10.220.217.17:81
    if source_str.startswith("http://") or source_str.startswith("https://"):
        if source_str.endswith(":81") or source_str.endswith(":81/"):
            return source_str.rstrip("/") + "/stream"
    return source_str

CAMERA_SOURCE = normalize_camera_source(CAMERA_SOURCE_RAW)

# MODEL SELECTION:
# - Set to "yolov8n.pt" to track everyday objects (person, cell phone, cup, bottle, etc.)
# - Set to "../models/bestmain.pt" to track the custom 3D printed target object
USE_STANDARD_MODEL = os.getenv("USE_STANDARD_MODEL", "false").lower() in ("true", "1", "yes")

if USE_STANDARD_MODEL:
    MODEL_PATH = os.getenv("TEST_MODEL_PATH", "yolov8n.pt")  # Standard YOLOv8 nano model
    TARGET_CLASS = os.getenv("TARGET_CLASS", "cell phone")
else:
    MODEL_PATH = os.getenv("MODEL_PATH", "../models/bestmain.pt")  # Custom trained model
    TARGET_CLASS = os.getenv("TARGET_CLASS", "target_object")

_model_path_obj = Path(MODEL_PATH)
if not _model_path_obj.is_absolute() and not _model_path_obj.exists():
    if (Path(__file__).resolve().parent / MODEL_PATH).exists():
        _model_path_obj = Path(__file__).resolve().parent / MODEL_PATH
    elif (Path(__file__).resolve().parent.parent / MODEL_PATH).exists():
        _model_path_obj = Path(__file__).resolve().parent.parent / MODEL_PATH
    elif (Path(__file__).resolve().parent.parent / "models" / Path(MODEL_PATH).name).exists():
        _model_path_obj = Path(__file__).resolve().parent.parent / "models" / Path(MODEL_PATH).name
MODEL_PATH = _model_path_obj

# Camera feed & Recognition resolution
FRAME_W = int(os.getenv("FRAME_W", "640"))
FRAME_H = int(os.getenv("FRAME_H", "480"))
IMG_SIZE = int(os.getenv("IMG_SIZE", "416"))   # Larger input = better small/fast object detection
CONFIDENCE = float(os.getenv("CONFIDENCE", "0.25"))  # Lower = catches blurry/partial fast-moving drones

# Deadband & Far zones for tracking calculation
HOLD_X = 35
FAR_X = 120
HOLD_Y = 30
FAR_Y = 100

KP_PAN_NEAR = 0.10
KP_PAN_FAR = 0.22
MAX_DELTA_PAN = 20
MIN_DELTA_PAN = 2

KP_TILT_NEAR = 0.075
KP_TILT_FAR = 0.18
MAX_DELTA_TILT = 14
MIN_DELTA_TILT = 3

ALPHA_X = 0.82           # Higher = snaps faster to new position (was 0.63)
ALPHA_Y = 0.80           # Higher = snaps faster to new position (was 0.60)
CMD_ALPHA_PAN = 0.80     # Faster servo command response (was 0.70)
CMD_ALPHA_TILT = 0.78    # Faster servo command response (was 0.66)
MAX_CMD_CHANGE_PAN = 10  # Allow bigger per-frame servo steps for fast targets (was 6)
MAX_CMD_CHANGE_TILT = 8  # Allow bigger per-frame servo steps for fast targets (was 5)
OSC_IGNORE_PAN = 3       # Less oscillation suppression for fast targets (was 5)
OSC_IGNORE_TILT = 3      # Less oscillation suppression for fast targets (was 4)

MIN_STABLE_DETECTIONS = 1   # Lock on in 1 frame — don't waste frames on fast re-entries (was 2)
MAX_CENTER_JUMP = 220       # Allow large position jumps for fast drones (was 120)
HIGH_CONF_ALLOW_JUMP = 0.40 # Allow jumps at lower confidence — motion blur drops conf (was 0.70)

# ==========================================
# HELPER FUNCTIONS
# ==========================================

def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))

def sign_of(val):
    return 1 if val > 0 else (-1 if val < 0 else 0)

def limit_change(new_val, old_val, max_chg):
    diff = new_val - old_val
    if diff > max_chg:
        return old_val + max_chg
    if diff < -max_chg:
        return old_val - max_chg
    return new_val

def anti_oscillation(new_cmd, old_cmd, ignore_thresh):
    if new_cmd == 0 or old_cmd == 0:
        return new_cmd
    if sign_of(new_cmd) != sign_of(old_cmd) and abs(new_cmd) <= ignore_thresh:
        return 0
    return new_cmd

def adaptive_delta(error, direction_sign, hold_zone, far_zone, kp_near, kp_far, max_delta, min_delta):
    abs_error = abs(error)
    if abs_error < hold_zone:
        return 0
    effective_error = abs_error - hold_zone
    kp = kp_far if abs_error >= far_zone else kp_near
    direction = sign_of(error)
    delta = direction_sign * direction * effective_error * kp
    delta = int(round(delta))
    if delta == 0:
        delta = direction_sign * direction * min_delta
    if abs(delta) < min_delta:
        delta = sign_of(delta) * min_delta
    return clamp(delta, -max_delta, max_delta)

def smooth_command(raw_cmd, last_cmd, cmd_alpha, max_chg, osc_ignore):
    raw_cmd = anti_oscillation(raw_cmd, last_cmd, osc_ignore)
    if raw_cmd == 0:
        return 0
    smoothed = int(round(last_cmd + cmd_alpha * (raw_cmd - last_cmd)))
    return limit_change(smoothed, last_cmd, max_chg)

import socket
import urllib.request
import numpy as np

# ==========================================
# LOW LATENCY CAMERA READER FOR NETWORK STREAMS
# ==========================================

class LatestFrameReader:
    def __init__(self, url):
        self.url = url
        self.host = None
        self.port = 81
        self.path = "/stream"

        if "://" in url:
            netloc = url.split("://", 1)[1]
            if "/" in netloc:
                host_port, self.path = netloc.split("/", 1)
                self.path = "/" + self.path
            else:
                host_port = netloc
            if ":" in host_port:
                self.host, port_str = host_port.split(":", 1)
                self.port = int(port_str)
            else:
                self.host = host_port
                self.port = 80
        else:
            self.host = url

        self.capture_url = f"http://{self.host}:80/capture"
        self.frame = None
        self.lock = threading.Lock()
        self.running = False
        self.thread = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _stream_socket(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(3.0)
        s.connect((self.host, self.port))
        req = f"GET {self.path} HTTP/1.1\r\nHost: {self.host}:{self.port}\r\nUser-Agent: Mozilla/5.0\r\nAccept: */*\r\nConnection: close\r\n\r\n"
        s.sendall(req.encode())

        buffer = b""
        while self.running:
            chunk = s.recv(4096)
            if not chunk:
                break
            buffer += chunk
            a = buffer.find(b"\xff\xd8")
            b = buffer.find(b"\xff\xd9")
            if a != -1 and b != -1:
                if b > a:
                    jpg = buffer[a:b+2]
                    buffer = buffer[b+2:]
                    frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is not None:
                        with self.lock:
                            self.frame = frame
                else:
                    buffer = buffer[a:]
        s.close()

    def _loop(self):
        while self.running:
            try:
                self._stream_socket()
            except Exception:
                # Fallback to /capture endpoint if streaming port is busy
                try:
                    req = urllib.request.Request(self.capture_url, headers={"User-Agent": "Mozilla/5.0"})
                    with urllib.request.urlopen(req, timeout=1.5) as resp:
                        arr = np.frombuffer(resp.read(), dtype=np.uint8)
                        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                        if frame is not None:
                            with self.lock:
                                self.frame = frame
                except Exception:
                    time.sleep(0.05)
            time.sleep(0.005)

    def read(self):
        with self.lock:
            if self.frame is None:
                return None
            return self.frame.copy()

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)

# ==========================================
# MAIN TEST LOOP
# ==========================================

def main():
    if isinstance(MODEL_PATH, Path) and not MODEL_PATH.exists():
        print(f"[ERROR] Model file not found at: {MODEL_PATH}")
        return

    print(f"Loading YOLO model: {MODEL_PATH} ...")
    model = YOLO(str(MODEL_PATH))
    print("Model loaded successfully!")
    print(f"Available classes: {list(model.names.values())[:10]} ...")
    if TARGET_CLASS:
        print(f"Tracking target: '{TARGET_CLASS}'")

    is_network_stream = isinstance(CAMERA_SOURCE, str) and (
        CAMERA_SOURCE.startswith("http://") or CAMERA_SOURCE.startswith("https://") or CAMERA_SOURCE.startswith("rtsp://")
    )

    cap = None
    reader = None

    if is_network_stream:
        print(f"\nConnecting to network camera stream: {CAMERA_SOURCE} ...")
        reader = LatestFrameReader(CAMERA_SOURCE)
        reader.start()
        print("Waiting for first camera frame (timeout 8s)...")
        first_frame = None
        for i in range(80):
            first_frame = reader.read()
            if first_frame is not None:
                break
            time.sleep(0.1)

        if first_frame is None:
            print(f"[ERROR] Could not receive video stream from {CAMERA_SOURCE}")
            print("[INFO] Please verify:")
            print("  1. ESP32-CAM is powered on and connected to your Wi-Fi.")
            print("  2. Your computer is on the same Wi-Fi / network subnet as the ESP32-CAM.")
            print(f"  3. Open {CAMERA_SOURCE} in a web browser to verify the stream.")
            reader.stop()
            return
        print("[SUCCESS] Video stream connected!")
    else:
        print(f"\nOpening local webcam index: {CAMERA_SOURCE} ...")
        # Use DirectShow backend on Windows to avoid MSMF hang
        backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
        cap = cv2.VideoCapture(int(CAMERA_SOURCE), backend)
        if not cap.isOpened():
            print(f"[ERROR] Could not open webcam index {CAMERA_SOURCE}.")
            print("[INFO] Check CAMERA_INDEX in .env or provide a stream URL like: python test_webcam_tracking.py http://10.220.217.17:81/stream")
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        print("[SUCCESS] Local webcam opened!")

    print("\n-----------------------------------------------------------")
    print("TEST MODE ACTIVE (Hardware / Serial is SIMULATED)")
    print(f"Camera Source: {CAMERA_SOURCE}")
    print("- Shows live detection with bounding box & confidence")
    print("- Shows simulated servo movement commands on screen & terminal")
    print("Controls:")
    print("  'q' - Quit test")
    print("  'c' - Simulate Center command")
    print("-----------------------------------------------------------\n")

    smooth_x = None
    smooth_y = None
    last_pan_cmd = 0
    last_tilt_cmd = 0
    stable_count = 0
    last_status_time = 0
    _loop_fps = 0.0
    _loop_last_t = time.time()

    # Velocity predictor — used to coast through brief missed frames
    vel_x = 0.0          # pixels/frame velocity
    vel_y = 0.0
    lost_frames = 0      # consecutive frames with no detection
    MAX_LOST_FRAMES = 5  # coast for up to 5 frames before giving up
    VEL_ALPHA = 0.55     # smoothing factor for velocity estimate (0=ignore, 1=raw)

    while True:
        if is_network_stream:
            frame = reader.read()
            if frame is None:
                time.sleep(0.01)
                continue
        else:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("[WARN] Failed to read frame from webcam.")
                time.sleep(0.05)
                continue

        frame = cv2.resize(frame, (FRAME_W, FRAME_H))
        h, w, _ = frame.shape
        center_x, center_y = w // 2, h // 2

        results = model.predict(frame, imgsz=IMG_SIZE, conf=CONFIDENCE, verbose=False)

        detected = False
        accepted = False
        pan_delta = 0
        tilt_delta = 0
        status = "IDLE"
        simulated_command = "None"

        # Find best matching detection (highest confidence match)
        best_box = None
        target_list = [c.strip().lower() for c in TARGET_CLASS.split(",")] if TARGET_CLASS else None
        allow_any = target_list is not None and ("any" in target_list or "all" in target_list)
        specific_targets = [c for c in target_list if c not in ("any", "all")] if target_list else []

        if results and len(results[0].boxes) > 0:
            matching_boxes = []
            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                cls_name = model.names.get(cls_id, str(cls_id)).lower()
                conf_val = float(box.conf[0])

                if specific_targets:
                    if cls_name in specific_targets:
                        matching_boxes.append(box)
                elif allow_any or not specific_targets:
                    matching_boxes.append(box)

            if matching_boxes:
                # Pick the match with the HIGHEST confidence score
                best_box = max(matching_boxes, key=lambda b: float(b.conf[0]))

            # Draw non-selected candidate boxes in faint color for live feedback
            for box in results[0].boxes:
                if box is not best_box:
                    bx1, by1, bx2, by2 = map(int, box.xyxy[0].tolist())
                    bconf = float(box.conf[0])
                    bcls_id = int(box.cls[0])
                    bcls_name = model.names.get(bcls_id, str(bcls_id))
                    cv2.rectangle(frame, (bx1, by1), (bx2, by2), (200, 180, 50), 1)
                    cv2.putText(frame, f"{bcls_name} ({bconf:.2f})", (bx1, max(15, by1 - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 180, 50), 1)

        if best_box is not None:
            x1, y1, x2, y2 = map(int, best_box.xyxy[0].tolist())
            conf = float(best_box.conf[0])
            cls_id = int(best_box.cls[0])
            cls_name = model.names.get(cls_id, str(cls_id))

            raw_x = (x1 + x2) // 2
            raw_y = (y1 + y2) // 2
            detected = True

            if smooth_x is not None:
                jump = ((raw_x - smooth_x)**2 + (raw_y - smooth_y)**2)**0.5
                if jump <= MAX_CENTER_JUMP or conf >= HIGH_CONF_ALLOW_JUMP:
                    accepted = True
            else:
                accepted = True

            if accepted:
                stable_count += 1
                if smooth_x is None:
                    smooth_x, smooth_y = raw_x, raw_y
                    vel_x, vel_y = 0.0, 0.0
                else:
                    # Update velocity estimate before moving smooth position
                    new_vel_x = float(raw_x - smooth_x)
                    new_vel_y = float(raw_y - smooth_y)
                    vel_x = VEL_ALPHA * new_vel_x + (1 - VEL_ALPHA) * vel_x
                    vel_y = VEL_ALPHA * new_vel_y + (1 - VEL_ALPHA) * vel_y

                    smooth_x = int(ALPHA_X * raw_x + (1 - ALPHA_X) * smooth_x)
                    smooth_y = int(ALPHA_Y * raw_y + (1 - ALPHA_Y) * smooth_y)

                lost_frames = 0  # reset coast counter on successful detection

                error_x = smooth_x - center_x
                error_y = smooth_y - center_y

                if stable_count < MIN_STABLE_DETECTIONS:
                    status = "LOCKING"
                    last_pan_cmd, last_tilt_cmd = 0, 0
                else:
                    raw_pan = adaptive_delta(error_x, -1, HOLD_X, FAR_X, KP_PAN_NEAR, KP_PAN_FAR, MAX_DELTA_PAN, MIN_DELTA_PAN)
                    raw_tilt = adaptive_delta(error_y, -1, HOLD_Y, FAR_Y, KP_TILT_NEAR, KP_TILT_FAR, MAX_DELTA_TILT, MIN_DELTA_TILT)

                    pan_delta = smooth_command(raw_pan, last_pan_cmd, CMD_ALPHA_PAN, MAX_CMD_CHANGE_PAN, OSC_IGNORE_PAN)
                    tilt_delta = smooth_command(raw_tilt, last_tilt_cmd, CMD_ALPHA_TILT, MAX_CMD_CHANGE_TILT, OSC_IGNORE_TILT)

                    last_pan_cmd, last_tilt_cmd = pan_delta, tilt_delta
                    status = "HOLD" if (pan_delta == 0 and tilt_delta == 0) else "TRACK"

                if pan_delta != 0 or tilt_delta != 0:
                    simulated_command = f"D,{pan_delta},{tilt_delta}"
                else:
                    simulated_command = "D,0,0 (In deadband / Hold)"

                # Draw Visuals
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"[TRACKING] {cls_name} ({conf:.2f})", (x1, max(20, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
                cv2.circle(frame, (smooth_x, smooth_y), 5, (0, 0, 255), -1)
            else:
                stable_count = 0
        else:
            # ── No detection this frame ──────────────────────────────────────
            if smooth_x is not None and lost_frames < MAX_LOST_FRAMES:
                # COAST: extrapolate position using last known velocity
                lost_frames += 1
                smooth_x = int(smooth_x + vel_x)
                smooth_y = int(smooth_y + vel_y)
                # Clamp to frame bounds
                smooth_x = max(0, min(w - 1, smooth_x))
                smooth_y = max(0, min(h - 1, smooth_y))

                # Keep issuing last servo command while coasting
                status = f"COASTING ({lost_frames}/{MAX_LOST_FRAMES})"
                simulated_command = f"D,{last_pan_cmd},{last_tilt_cmd}"

                # Draw coasting indicator (dashed orange circle)
                cv2.circle(frame, (smooth_x, smooth_y), 10, (0, 140, 255), 2)
                cv2.putText(frame, f"COASTING", (smooth_x + 12, smooth_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 140, 255), 1)
            else:
                # Lost for too long — full reset
                smooth_x, smooth_y = None, None
                vel_x, vel_y = 0.0, 0.0
                lost_frames = 0
                last_pan_cmd, last_tilt_cmd = 0, 0
                stable_count = 0
                status = "NO TARGET"

        # Center frame crosshair & hold zone rectangle
        cv2.drawMarker(frame, (center_x, center_y), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)
        cv2.rectangle(frame, (center_x - HOLD_X, center_y - HOLD_Y),
                      (center_x + HOLD_X, center_y + HOLD_Y), (255, 255, 0), 1)

        # Status text overlay
        cv2.putText(frame, f"STATUS: {status}", (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255) if status == "TRACK" else (200, 200, 200), 2)
        cv2.putText(frame, f"Simulated Servo Cmd: {simulated_command}", (15, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0) if "D," in simulated_command and simulated_command != "D,0,0" else (180, 180, 180), 2)

        # ── Push to AstraNex UI (non-blocking) ──────────────────────────────
        _now_t = time.time()
        _dt = _now_t - _loop_last_t
        _loop_last_t = _now_t
        if _dt > 0:
            _loop_fps = 0.9 * _loop_fps + 0.1 * (1.0 / _dt)
        if UI_ENABLED:
            _bbox_val = (x1, y1, x2, y2) if best_box is not None else None
            _cls_val  = cls_name if best_box is not None else ''
            _conf_val = conf if best_box is not None else 0.0
            push_state(
                status=status,
                cls_name=_cls_val,
                confidence=_conf_val,
                pan_delta=pan_delta,
                tilt_delta=tilt_delta,
                error_x=int(smooth_x - center_x) if smooth_x is not None else 0,
                error_y=int(smooth_y - center_y) if smooth_y is not None else 0,
                smooth_x=smooth_x,
                smooth_y=smooth_y,
                bbox=_bbox_val,
                fps=round(_loop_fps, 1),
                frame_w=FRAME_W,
                frame_h=FRAME_H,
            )
            # Encode frame as JPEG and push
            _ok, _jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if _ok:
                _push_frame(bytes(_jpg))

        cv2.imshow("ESP32-CAM AI Tracker [TEST MODE - WEBCAM]", frame)

        # Print periodic log to terminal
        if time.time() - last_status_time > 1.0:
            if detected and accepted:
                print(f"[TEST] Status: {status:7s} | Cmd -> ESP32: {simulated_command}")
            else:
                print(f"[TEST] Status: {status}")
            last_status_time = time.time()

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            print("[TEST] Sent Center Command ('C') to simulated ESP32")
            smooth_x, smooth_y = None, None
            last_pan_cmd, last_tilt_cmd = 0, 0

    if reader is not None:
        reader.stop()
    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()
    print("Test finished.")

if __name__ == "__main__":
    main()

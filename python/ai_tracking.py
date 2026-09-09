import os
import cv2
import serial
import time
import threading
import math
import socket
import urllib.request
import numpy as np
from pathlib import Path
from dotenv import load_dotenv
from ultralytics import YOLO

# Load .env from python directory or project root
load_dotenv(dotenv_path=Path(__file__).resolve().parent / ".env")
load_dotenv(dotenv_path=Path(__file__).resolve().parent.parent / ".env")

# SETTINGS

# Path to the trained YOLO model
_raw_model_path = os.getenv("MODEL_PATH", "../models/bestmain.pt")
_model_path_obj = Path(_raw_model_path)
if not _model_path_obj.is_absolute() and not _model_path_obj.exists():
    if (Path(__file__).resolve().parent / _raw_model_path).exists():
        _model_path_obj = Path(__file__).resolve().parent / _raw_model_path
    elif (Path(__file__).resolve().parent.parent / _raw_model_path).exists():
        _model_path_obj = Path(__file__).resolve().parent.parent / _raw_model_path
    elif (Path(__file__).resolve().parent.parent / "models" / Path(_raw_model_path).name).exists():
        _model_path_obj = Path(__file__).resolve().parent.parent / "models" / Path(_raw_model_path).name
MODEL_PATH = str(_model_path_obj)

# Example on macOS: "/dev/cu.usbserial-XXXX"
# Example on Windows: "COM3"
# Example on Linux / Docker: "/dev/ttyUSB0"
SERIAL_PORT = os.getenv("SERIAL_PORT", "YOUR_SERIAL_PORT")

# Replace with your ESP32-CAM stream URL
CAMERA_URL = os.getenv("CAMERA_URL", "http://YOUR_ESP32_CAM_IP:81/stream")

# Set HEADLESS=true in Docker or server environments without a display
HEADLESS = os.getenv("HEADLESS", "false").lower() in ("true", "1", "yes")

PAN_SIGN = int(os.getenv("PAN_SIGN", "-1"))
TILT_SIGN = int(os.getenv("TILT_SIGN", "-1"))

# Recognition settings
IMG_SIZE = 320
CONFIDENCE = float(os.getenv("CONFIDENCE", "0.35"))
TARGET_CLASS = os.getenv("TARGET_CLASS", "person")

FRAME_W = 320
FRAME_H = 240

# MOVEMENT SETTINGS

# PAN / LEFT-RIGHT

HOLD_X = 22
FAR_X = 80

KP_PAN_NEAR = 0.10
KP_PAN_FAR = 0.22

MAX_DELTA_PAN = 20
MIN_DELTA_PAN = 2

# TILT / UP-DOWN

HOLD_Y = 30
FAR_Y = 58

KP_TILT_NEAR = 0.075
KP_TILT_FAR = 0.18

MAX_DELTA_TILT = 14
MIN_DELTA_TILT = 3

COMMAND_DELAY = 0.030


CENTER_HOLD_SECONDS = 1.25


ALPHA_X = 0.63
ALPHA_Y = 0.60

CMD_ALPHA_PAN = 0.70
CMD_ALPHA_TILT = 0.66

MAX_CMD_CHANGE_PAN = 6
MAX_CMD_CHANGE_TILT = 5


OSC_IGNORE_PAN = 5
OSC_IGNORE_TILT = 4


MIN_STABLE_DETECTIONS = 2
MAX_CENTER_JUMP = 90
HIGH_CONF_ALLOW_JUMP = 0.70

PRINT_COMMANDS = False

# GLOBAL STATE

smooth_x = None
smooth_y = None

last_pan_cmd = 0
last_tilt_cmd = 0

last_command_time = 0
last_status_time = 0
last_sent_move = False

stable_detection_count = 0

center_hold_until = 0.0

# LOW LATENCY CAMERA READER

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

# FUNCTIONS

def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def sign_of(value):
    if value > 0:
        return 1
    if value < 0:
        return -1
    return 0


def limit_change(new_value, old_value, max_change):
    diff = new_value - old_value

    if diff > max_change:
        return old_value + max_change

    if diff < -max_change:
        return old_value - max_change

    return new_value


def anti_oscillation(new_cmd, old_cmd, ignore_threshold):
    if new_cmd == 0:
        return 0

    if old_cmd == 0:
        return new_cmd

    if sign_of(new_cmd) != sign_of(old_cmd) and abs(new_cmd) <= ignore_threshold:
        return 0

    return new_cmd


def adaptive_delta(
    error,
    direction_sign,
    hold_zone,
    far_zone,
    kp_near,
    kp_far,
    max_delta,
    min_delta
):
    abs_error = abs(error)

    if abs_error < hold_zone:
        return 0

    effective_error = abs_error - hold_zone

    if abs_error >= far_zone:
        kp = kp_far
    else:
        kp = kp_near

    error_direction = sign_of(error)

    delta = direction_sign * error_direction * effective_error * kp
    delta = int(round(delta))

    if delta == 0:
        delta = direction_sign * error_direction * min_delta

    if abs(delta) < min_delta:
        delta = sign_of(delta) * min_delta

    return clamp(delta, -max_delta, max_delta)


def smooth_command(raw_cmd, last_cmd, cmd_alpha, max_change, osc_ignore):
    raw_cmd = anti_oscillation(raw_cmd, last_cmd, osc_ignore)

    if raw_cmd == 0:
        return 0

    smoothed = last_cmd + cmd_alpha * (raw_cmd - last_cmd)
    smoothed = int(round(smoothed))

    smoothed = limit_change(smoothed, last_cmd, max_change)

    return smoothed


def reset_tracking_state():
    global smooth_x, smooth_y
    global last_pan_cmd, last_tilt_cmd
    global stable_detection_count, last_sent_move

    smooth_x = None
    smooth_y = None

    last_pan_cmd = 0
    last_tilt_cmd = 0

    stable_detection_count = 0
    last_sent_move = False

# SETUP

print("Loading YOLO model...")
model = YOLO(MODEL_PATH)

print(f"Opening serial port ({SERIAL_PORT})...")
try:
    ser = serial.Serial(SERIAL_PORT, 115200, timeout=1)
except Exception as e:
    import serial.tools.list_ports as lp
    available_ports = [p.device for p in lp.comports()]
    print(f"\n[ERROR] Failed to open serial port '{SERIAL_PORT}': {e}")
    if available_ports:
        print(f"[INFO] Available COM ports detected: {', '.join(available_ports)}")
        print(f"[INFO] Update SERIAL_PORT in your .env file to one of the above.")
    else:
        print("[INFO] No COM ports detected. Please ensure your ESP32 controller is plugged in via USB and drivers (CH340 / CP210x) are installed.")
        print("[INFO] If you want to test tracking with your webcam without hardware, run: python test_webcam_tracking.py\n")
    exit(1)
time.sleep(2)

ser.reset_input_buffer()
ser.reset_output_buffer()

print("Opening ESP32-CAM stream...")
reader = LatestFrameReader(CAMERA_URL)
reader.start()

print("Waiting for camera frame...")
first_frame = None

for _ in range(100):
    first_frame = reader.read()
    if first_frame is not None:
        break
    time.sleep(0.05)

if first_frame is None:
    print("Could not get camera frame.")
    ser.close()
    reader.stop()
    exit()

print("YOLO tracking started.")
print("c = smooth center servos")
print("q = quit")
print("Click the OpenCV video window before pressing c or q.")

ser.write(b"C\n")
center_hold_until = time.time() + CENTER_HOLD_SECONDS
time.sleep(0.5)

# MAIN LOOP

while True:
    frame = reader.read()

    if frame is None:
        print("Camera frame not found")
        time.sleep(0.02)
        continue

    frame = cv2.resize(frame, (FRAME_W, FRAME_H))

    h, w, _ = frame.shape
    frame_center_x = w // 2
    frame_center_y = h // 2

    results = model.predict(
        frame,
        imgsz=IMG_SIZE,
        conf=CONFIDENCE,
        verbose=False
    )

    detected = False
    accepted_detection = False
    pan_delta = 0
    tilt_delta = 0
    status = "NO TARGET"
    cls_name = ""

    best_box = None
    target_list = [c.strip().lower() for c in TARGET_CLASS.split(",")] if TARGET_CLASS else None
    allow_any = target_list is not None and ("any" in target_list or "all" in target_list)
    specific_targets = [c for c in target_list if c not in ("any", "all")] if target_list else []

    if results and len(results[0].boxes) > 0:
        matching_boxes = []
        for b in results[0].boxes:
            cid = int(b.cls[0])
            cname = model.names.get(cid, str(cid)).lower()
            if specific_targets:
                if cname in specific_targets:
                    matching_boxes.append(b)
            elif allow_any or not specific_targets:
                matching_boxes.append(b)

        if matching_boxes:
            best_box = max(matching_boxes, key=lambda b: float(b.conf[0]))

    if best_box is not None:
        x1, y1, x2, y2 = best_box.xyxy[0].cpu().numpy()
        conf = float(best_box.conf[0].cpu().numpy())
        cls_id = int(best_box.cls[0].cpu().numpy())
        cls_name = model.names.get(cls_id, str(cls_id))

        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

        raw_x = (x1 + x2) // 2
        raw_y = (y1 + y2) // 2

        detected = True

        if smooth_x is not None and smooth_y is not None:
            jump = math.sqrt((raw_x - smooth_x) ** 2 + (raw_y - smooth_y) ** 2)

            if jump > MAX_CENTER_JUMP and conf < HIGH_CONF_ALLOW_JUMP:
                status = "JUMP IGNORED"
                accepted_detection = False
            else:
                accepted_detection = True
        else:
            accepted_detection = True

        if accepted_detection:
            stable_detection_count += 1

            if smooth_x is None:
                smooth_x = raw_x
                smooth_y = raw_y
            else:
                smooth_x = int(ALPHA_X * raw_x + (1 - ALPHA_X) * smooth_x)
                smooth_y = int(ALPHA_Y * raw_y + (1 - ALPHA_Y) * smooth_y)

            error_x = smooth_x - frame_center_x
            error_y = smooth_y - frame_center_y

            # Wait for stable detection before moving
            if stable_detection_count < MIN_STABLE_DETECTIONS:
                pan_delta = 0
                tilt_delta = 0
                last_pan_cmd = 0
                last_tilt_cmd = 0
                status = "LOCKING"
            else:
                raw_pan_delta = adaptive_delta(
                    error=error_x,
                    direction_sign=PAN_SIGN,
                    hold_zone=HOLD_X,
                    far_zone=FAR_X,
                    kp_near=KP_PAN_NEAR,
                    kp_far=KP_PAN_FAR,
                    max_delta=MAX_DELTA_PAN,
                    min_delta=MIN_DELTA_PAN
                )

                raw_tilt_delta = adaptive_delta(
                    error=error_y,
                    direction_sign=TILT_SIGN,
                    hold_zone=HOLD_Y,
                    far_zone=FAR_Y,
                    kp_near=KP_TILT_NEAR,
                    kp_far=KP_TILT_FAR,
                    max_delta=MAX_DELTA_TILT,
                    min_delta=MIN_DELTA_TILT
                )

                pan_delta = smooth_command(
                    raw_pan_delta,
                    last_pan_cmd,
                    CMD_ALPHA_PAN,
                    MAX_CMD_CHANGE_PAN,
                    OSC_IGNORE_PAN
                )

                tilt_delta = smooth_command(
                    raw_tilt_delta,
                    last_tilt_cmd,
                    CMD_ALPHA_TILT,
                    MAX_CMD_CHANGE_TILT,
                    OSC_IGNORE_TILT
                )

                last_pan_cmd = pan_delta
                last_tilt_cmd = tilt_delta

                if pan_delta == 0 and tilt_delta == 0:
                    status = "HOLD"
                else:
                    status = "TRACK"


            cv2.circle(frame, (smooth_x, smooth_y), 5, (0, 0, 255), -1)

        else:
            stable_detection_count = 0
            last_pan_cmd = 0
            last_tilt_cmd = 0
            pan_delta = 0
            tilt_delta = 0

  
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)


        cv2.circle(frame, (raw_x, raw_y), 4, (0, 255, 255), -1)


        cv2.circle(frame, (frame_center_x, frame_center_y), 5, (255, 255, 255), -1)


        cv2.rectangle(
            frame,
            (frame_center_x - HOLD_X, frame_center_y - HOLD_Y),
            (frame_center_x + HOLD_X, frame_center_y + HOLD_Y),
            (255, 255, 0),
            1
        )

        cv2.putText(
            frame,
            f"[{cls_name}] {status} {conf:.2f} D=({pan_delta},{tilt_delta})",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 255),
            1
        )

    else:
        smooth_x = None
        smooth_y = None
        last_pan_cmd = 0
        last_tilt_cmd = 0
        stable_detection_count = 0

        cv2.putText(
            frame,
            "No target detected",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            2
        )

    now = time.time()


    # Command sending


    if now < center_hold_until:
        last_pan_cmd = 0
        last_tilt_cmd = 0
        last_sent_move = False

    elif detected and accepted_detection and now - last_command_time > COMMAND_DELAY:
        if pan_delta != 0 or tilt_delta != 0:
            command = f"D,{pan_delta},{tilt_delta}\n"
            ser.write(command.encode())
            last_sent_move = True

            if PRINT_COMMANDS:
                print(command.strip())

        elif last_sent_move:
            ser.write(b"D,0,0\n")
            last_sent_move = False

        last_command_time = now

    elif not accepted_detection and last_sent_move and now - last_command_time > COMMAND_DELAY:
        ser.write(b"D,0,0\n")
        last_sent_move = False
        last_command_time = now

    if now - last_status_time > 1.0:
        if now < center_hold_until:
            print("CENTERING")
        elif detected:
            print(f"{status} | D=({pan_delta},{tilt_delta})")
        else:
            print("No target")
        last_status_time = now

    if not HEADLESS:
        cv2.imshow("YOLO Object Tracking - SMOOTH CENTER", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("c"):
            ser.write(b"C\n")
            reset_tracking_state()
            center_hold_until = time.time() + CENTER_HOLD_SECONDS
            print("CENTER")

        elif key == ord("q"):
            break
    else:
        # Prevent spinning in 100% CPU loop when headless
        time.sleep(0.005)


try:
    ser.write(b"C\n")
    time.sleep(0.3)
except Exception:
    pass

reader.stop()
ser.close()
if not HEADLESS:
    cv2.destroyAllWindows()
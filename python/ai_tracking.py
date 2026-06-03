import cv2
import serial
import time
import threading
import math
from ultralytics import YOLO

# SETTINGS

# Path to the trained YOLO model
MODEL_PATH = "../models/best.pt"

# Example on macOS: "/dev/cu.usbserial-XXXX"
# Example on Windows: "COM3"
SERIAL_PORT = "YOUR_SERIAL_PORT"

# Replace with your ESP32-CAM stream URL
CAMERA_URL = "http://YOUR_ESP32_CAM_IP:81/stream"

PAN_SIGN = -1
TILT_SIGN = -1

# Recognition settings
IMG_SIZE = 320
CONFIDENCE = 0.35

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
        self.cap = None
        self.frame = None
        self.lock = threading.Lock()
        self.running = False
        self.thread = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _open(self):
        if self.cap is not None:
            self.cap.release()

        self.cap = cv2.VideoCapture(self.url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def _loop(self):
        self._open()

        while self.running:
            if self.cap is None or not self.cap.isOpened():
                self._open()
                time.sleep(0.05)
                continue

            ret, frame = self.cap.read()

            if ret and frame is not None:
                with self.lock:
                    self.frame = frame

            time.sleep(0.001)

    def read(self):
        with self.lock:
            if self.frame is None:
                return None
            return self.frame.copy()

    def stop(self):
        self.running = False

        if self.thread is not None:
            self.thread.join(timeout=1.0)

        if self.cap is not None:
            self.cap.release()

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

print("Opening serial port...")
ser = serial.Serial(SERIAL_PORT, 115200, timeout=1)
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
        max_det=1,
        verbose=False
    )

    detected = False
    accepted_detection = False
    pan_delta = 0
    tilt_delta = 0
    status = "NO TARGET"

    if (
        len(results) > 0
        and results[0].boxes is not None
        and len(results[0].boxes) > 0
    ):
        box = results[0].boxes[0]

        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
        conf = float(box.conf[0].cpu().numpy())

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
            f"{status} {conf:.2f} D=({pan_delta},{tilt_delta})",
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

    cv2.imshow("YOLO Object Tracking - SMOOTH CENTER", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("c"):
        ser.write(b"C\n")
        reset_tracking_state()
        center_hold_until = time.time() + CENTER_HOLD_SECONDS
        print("CENTER")

    elif key == ord("q"):
        break


try:
    ser.write(b"C\n")
    time.sleep(0.3)
except Exception:
    pass

reader.stop()
ser.close()
cv2.destroyAllWindows()
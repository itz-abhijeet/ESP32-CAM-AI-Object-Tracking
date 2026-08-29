import cv2
import time
import os
from pathlib import Path
from dotenv import load_dotenv
from ultralytics import YOLO

# Load .env from python directory or project root
load_dotenv(dotenv_path=Path(__file__).resolve().parent / ".env")
load_dotenv(dotenv_path=Path(__file__).resolve().parent.parent / ".env")

# ==========================================
# TEST SETTINGS (NO HARDWARE REQUIRED)
# ==========================================

# 0 is usually the default laptop/USB webcam
CAMERA_INDEX = int(os.getenv("CAMERA_INDEX", "0"))

# MODEL SELECTION:
# - Set to "yolov8n.pt" to track everyday objects (person, cell phone, cup, bottle, etc.)
# - Set to "../models/best.pt" to track the custom 3D printed target object
USE_STANDARD_MODEL = os.getenv("USE_STANDARD_MODEL", "true").lower() in ("true", "1", "yes")

if USE_STANDARD_MODEL:
    MODEL_PATH = os.getenv("TEST_MODEL_PATH", "yolov8n.pt")  # Standard YOLOv8 nano model
    TARGET_CLASS = os.getenv("TARGET_CLASS", "cell phone")
else:
    MODEL_PATH = os.getenv("MODEL_PATH", "../models/best.pt")  # Custom trained model
    TARGET_CLASS = os.getenv("TARGET_CLASS", "target_object")

# Camera feed & Recognition resolution
FRAME_W = int(os.getenv("FRAME_W", "640"))
FRAME_H = int(os.getenv("FRAME_H", "480"))
IMG_SIZE = int(os.getenv("IMG_SIZE", "320"))
CONFIDENCE = float(os.getenv("CONFIDENCE", "0.35"))

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

ALPHA_X = 0.63
ALPHA_Y = 0.60
CMD_ALPHA_PAN = 0.70
CMD_ALPHA_TILT = 0.66
MAX_CMD_CHANGE_PAN = 6
MAX_CMD_CHANGE_TILT = 5
OSC_IGNORE_PAN = 5
OSC_IGNORE_TILT = 4

MIN_STABLE_DETECTIONS = 2
MAX_CENTER_JUMP = 120
HIGH_CONF_ALLOW_JUMP = 0.70

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

    print(f"\nOpening webcam index {CAMERA_INDEX}...")
    cap = cv2.VideoCapture(CAMERA_INDEX)

    if not cap.isOpened():
        print("[ERROR] Could not open webcam. If you don't have a webcam or it's in use, check CAMERA_INDEX.")
        return

    print("\n-----------------------------------------------------------")
    print("TEST MODE ACTIVE (Hardware / Serial is SIMULATED)")
    print("- Shows live webcam detection with bounding box & confidence")
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

    while True:
        ret, frame = cap.read()
        if not ret:
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
                else:
                    smooth_x = int(ALPHA_X * raw_x + (1 - ALPHA_X) * smooth_x)
                    smooth_y = int(ALPHA_Y * raw_y + (1 - ALPHA_Y) * smooth_y)

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
            smooth_x, smooth_y = None, None
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

    cap.release()
    cv2.destroyAllWindows()
    print("Test finished.")

if __name__ == "__main__":
    main()

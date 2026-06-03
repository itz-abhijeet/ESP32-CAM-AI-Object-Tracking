import cv2
import os
import time

# Replace this with your own ESP32-CAM stream URL
STREAM_URL = "http://YOUR_ESP32_CAM_IP:81/stream"

# Folder where captured training images will be saved
SAVE_DIR = "raw_images"

os.makedirs(SAVE_DIR, exist_ok=True)

cap = cv2.VideoCapture(STREAM_URL)

if not cap.isOpened():
    print("Could not open ESP32-CAM stream. Check the IP address.")
    exit()

saved = 0

print("Controls:")
print("s = save current frame")
print("a = auto-save mode on/off")
print("q = quit")

auto_save = False
last_save_time = 0
auto_interval = 1.0

while True:
    ret, frame = cap.read()

    if not ret:
        print("Frame not received. Retrying...")
        time.sleep(0.3)
        continue

    cv2.imshow("ESP32-CAM Dataset Capture", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("s"):
        filename = os.path.join(SAVE_DIR, f"img_{saved:04d}.jpg")
        cv2.imwrite(filename, frame)
        print("Saved:", filename)
        saved += 1

    elif key == ord("a"):
        auto_save = not auto_save
        print("Auto-save:", "ON" if auto_save else "OFF")

    if auto_save and time.time() - last_save_time > auto_interval:
        filename = os.path.join(SAVE_DIR, f"img_{saved:04d}.jpg")
        cv2.imwrite(filename, frame)
        print("Auto-saved:", filename)
        saved += 1
        last_save_time = time.time()

    if key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()

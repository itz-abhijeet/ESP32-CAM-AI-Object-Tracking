# Optical Drone Detection & Tracking Setup Guide

This guide explains how to connect and configure this project hardware and software specifically for optical drone (UAV) detection and tracking using computer vision and standard pan-tilt mounts.

---

## 1. System Architecture

```text
[ ESP32-CAM ] 
      │ (Wi-Fi MJPEG Stream)
      ▼
[ Laptop / PC ] ── (Runs YOLO Detection + PID Tracking)
      │ (USB Serial: Movement Commands)
      ▼
[ ESP32 Controller ] 
      │ (I2C)
      ▼
[ PCA9685 Driver ] ── (PWM) ──> [ Pan / Tilt Servos ]
```

---

## 2. Hardware Connections

### Wiring Diagram

1. **Second ESP32 to PCA9685 Driver (I2C Control):**
   * `ESP32 GPIO 21 (SDA)` → `PCA9685 SDA`
   * `ESP32 GPIO 22 (SCL)` → `PCA9685 SCL`
   * `ESP32 5V / VIN` → `PCA9685 VCC` (Logic Power)
   * `ESP32 GND` → `PCA9685 GND`

2. **PCA9685 to Servos & External Power:**
   * **Pan Servo:** Plug into **Channel 0** on the PCA9685.
   * **Tilt Servo:** Plug into **Channel 1** on the PCA9685.
   * **External 5V Power Supply:** Connect to the `V+` and `GND` screw terminals on the PCA9685.

> **Important:** Always connect the ground (`GND`) of the external power supply and the ESP32 together to maintain a common ground reference. Never draw servo operating current directly from the ESP32 board pins.

---

## 3. Firmware Flashing

### A. ESP32-CAM (Video Streaming)
1. Open `arduino/esp32_cam_stream/esp32_cam_stream.ino` in the Arduino IDE.
2. Update your local Wi-Fi details:
   ```cpp
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```
3. Set the board to **AI Thinker ESP32-CAM** and upload.
4. Open the Serial Monitor (115200 baud) to obtain the device's IP address (e.g., `http://192.168.1.50:81/stream`).

### B. Second ESP32 (Pan/Tilt Motion Controller)
1. Open `arduino/servo_controller/servo_controller.ino` in the Arduino IDE.
2. Select your board (e.g., **ESP32 Dev Module**) and target COM port.
3. Upload the firmware. Leave the ESP32 connected to the PC via USB.

---

## 4. Software Configuration for Drone Detection

### Model Selection
Standard YOLO models (such as YOLOv8) can detect commercial drones or aerial objects using custom datasets or classes such as `airplane` or custom-trained drone weights (`.pt`).

### Updating `python/ai_tracking.py`
Edit the configuration variables in `python/ai_tracking.py`:

```python
# Path to your YOLO model trained on drone imagery
MODEL_PATH = "yolov8n.pt"  # Or your custom drone detection model: "models/drone_model.pt"

# Serial port connected to the Second ESP32
SERIAL_PORT = "COM3"  # Adjust based on your Device Manager / COM port

# ESP32-CAM stream URL obtained from Serial Monitor
CAMERA_URL = "http://192.168.1.50:81/stream"
```

---

## 5. Running the Tracker

1. Power on the servos and ESP32-CAM.
2. Run the main tracking script:
   ```bash
   cd python
   python ai_tracking.py
   ```
3. The optical tracker will stream the video feed, detect the aerial target, compute position offsets from the frame center, and send real-time pan/tilt correction commands to maintain visual lock.

# Testing Checklist

This checklist was used to test the ESP32-CAM AI object-tracking prototype as a full system.

The full system flow is:

```text
ESP32-CAM video stream
→ Python + YOLO detection
→ object center calculation
→ serial movement command
→ second ESP32
→ PCA9685 servo driver
→ pan/tilt servo movement
→ camera follows the target
```

This project is still a prototype, so the checklist focuses on behavior, stability, safety, and repeatability.

---

## 1. Startup Test

**Goal:** Make sure the system starts cleanly.

**Steps:**

1. Power the ESP32-CAM.
2. Power the second ESP32 and servo driver.
3. Start the Python tracking script.
4. Check the terminal, OpenCV window, and physical camera movement.

**Expected result:**

* YOLO model loads successfully.
* Serial port opens.
* ESP32-CAM stream opens.
* OpenCV window appears.
* Camera receives the center command.
* Servos move to the center position smoothly.
* No sudden movement or servo buzzing.

---

## 2. Center Command Test

**Goal:** Make sure the `C` command centers the camera correctly.

**Steps:**

1. Start tracking.
2. Let the camera follow the target.
3. Press `c` in the OpenCV window.

**Expected result:**

* Camera returns to center smoothly.
* Old movement commands are cleared.
* Tracking resumes normally after centering.
* Camera does not jump back to an old target position.

---

## 3. Quit Behavior Test

**Goal:** Make sure the program exits cleanly.

**Steps:**

1. Start tracking.
2. Let the camera move.
3. Press `q`.

**Expected result:**

* Camera sends a center command before exit.
* OpenCV window closes.
* Serial port closes.
* Python process exits.
* The next launch works without a “port busy” error.

---

## 4. No Target Test

**Goal:** Make sure the camera stays stable when the target is not visible.

**Steps:**

1. Remove the target object from the camera view.
2. Watch the OpenCV window and physical camera.

**Expected result:**

* Camera stops moving.
* No random searching behavior.
* No tracking of the background.
* OpenCV window shows “No target detected.”
* Terminal shows “No target.”

---

## 5. Lost Target While Moving Test

**Goal:** Make sure the camera stops when the target disappears during movement.

**Steps:**

1. Put the target object on one side of the frame.
2. Let the camera begin moving toward it.
3. Quickly remove the object from the frame.

**Expected result:**

* Camera stops shortly after the target disappears.
* Camera does not continue moving toward the old target position.
* No delayed movement or runaway behavior.

---

## 6. Target Reacquisition Test

**Goal:** Make sure tracking restarts correctly when the object returns.

**Steps:**

1. Remove the target for 2–3 seconds.
2. Put the target back into the frame.

**Expected result:**

* System may briefly show `LOCKING`.
* Tracking resumes after stable detection.
* Camera does not jump suddenly to an old position.
* Camera follows the target again normally.

---

## 7. Pan Direction Test

**Goal:** Verify left/right movement direction.

**Steps:**

1. Put the target near the center.
2. Move the target slowly to the right.
3. Move the target slowly to the left.

**Expected result:**

* Target moves right → camera pans right.
* Target moves left → camera pans left.
* Camera moves toward the target, not away from it.

---

## 8. Tilt Direction Test

**Goal:** Verify up/down movement direction.

**Steps:**

1. Put the target near the center.
2. Move the target upward.
3. Move the target downward.

**Expected result:**

* Target moves up → camera tilts up.
* Target moves down → camera tilts down.
* Tilt axis does not move in the opposite direction.

---

## 9. Hold Zone Test

**Goal:** Make sure the camera does not twitch when the object is already near the center.

**Steps:**

1. Put the object close to the center of the frame.
2. Move it slightly inside the hold-zone rectangle.

**Expected result:**

* Camera stays still.
* Status shows `HOLD`.
* Movement commands stay near `D=(0,0)`.
* No small left/right or up/down twitching.

---

## 10. Smooth Tracking Test

**Goal:** Check normal smooth tracking behavior.

**Steps:**

1. Move the target slowly from left to right.
2. Move it from right to left.
3. Move it up and down.

**Expected result:**

* Camera follows smoothly.
* Movement does not look shaky.
* Object stays near the center.
* No repeated stop-go-stop-go behavior.

---

## 11. Fast Movement Test

**Goal:** Check how the system reacts when the object moves quickly.

**Steps:**

1. Start with the target in the center.
2. Move it quickly to the side.
3. Move it quickly up or down.

**Expected result:**

* Camera reacts without going out of control.
* Camera does not overshoot too much.
* Movement settles after the object stops.
* No continuous oscillation.

---

## 12. False Positive Test

**Goal:** Make sure the camera does not follow random objects.

**Steps:**

1. Remove the target object.
2. Place similar-looking objects or hands in the frame.
3. Change the background or lighting.

**Expected result:**

* YOLO does not confidently track the wrong object.
* Camera does not aggressively follow the background.
* If false detections happen, they should be limited and not cause unstable movement.

---

## 13. Latency Test

**Goal:** Check if the camera is reacting to recent frames instead of old buffered frames.

**Steps:**

1. Move the target quickly in front of the camera.
2. Watch the OpenCV window.
3. Watch the physical servo response.

**Expected result:**

* OpenCV window shows the recent target position.
* Camera does not follow where the target was one second ago.
* Tracking delay is acceptable for a prototype.

---

## 14. Edge Limit Test

**Goal:** Make sure servos stay within safe physical limits.

**Steps:**

1. Move the target far left and far right.
2. Move the target high and low.
3. Watch the servo and 3D-printed parts.

**Expected result:**

* Camera does not hit the plastic frame.
* Servo horn does not push against the mount.
* Camera cable does not get pulled.
* No buzzing at servo limits.

---

## 15. Arduino Timeout Test

**Goal:** Make sure the servo controller stops safely if Python stops sending commands.

**Steps:**

1. Start tracking.
2. Let the camera move.
3. Stop or crash the Python script.

**Expected result:**

* Arduino stops updating movement after command timeout.
* Camera does not keep moving forever.
* Servo controller does not freeze.
* System can be restarted normally.

---

## 16. Command Protocol Test

**Goal:** Verify Python-to-Arduino serial commands.

**Command format:**

```text
C
D,panDelta,tiltDelta
```

**Expected result:**

* `C` centers the camera.
* `D,x,y` moves pan and tilt.
* `D,0,0` stops movement.
* Commands do not look chaotic when the target is near the center.

---

## 17. Long Run Test

**Goal:** Check stability over time.

**Steps:**

1. Run the project for 15–30 minutes.
2. Move the target occasionally.
3. Remove and return the target.
4. Press `c` occasionally to center.

**Expected result:**

* Python does not crash.
* Camera stream does not freeze.
* Serial connection stays active.
* Servos do not overheat.
* Tracking behavior stays consistent.

---

## 18. Thermal Test

**Goal:** Check if the electronics and servos stay within a safe temperature range.

**Steps:**
After a longer run, check:

* ESP32-CAM
* second ESP32
* PCA9685
* pan servo
* tilt servo

**Expected result:**

* Warm is expected.
* Very hot or too hot to touch is not good.
* If a servo is very hot, check mechanical friction, servo limits, weight balance, and power supply.

---

## 19. Repeatability Test

**Goal:** Make sure the project works consistently across multiple launches.

**Steps:**
Repeat 5 times:

1. Start Python.
2. Center the camera.
3. Track left/right.
4. Track up/down.
5. Remove target.
6. Return target.
7. Press `c`.
8. Press `q`.

**Expected result:**

* Behavior is consistent across launches.
* Serial port is released correctly.
* Camera centers correctly every time.
* Tracking does not become worse after restart.

---

## 20. Demo Quality Test

**Goal:** Check if the project is understandable in a short demo video.

**Expected result:**

* Within the first few seconds, it is clear that the camera follows the object.
* Movement looks smooth enough for a prototype.
* The object is visible.
* The camera does not look confused.
* The wiring and mechanical parts look intentional.
* The demo can be understood without a long explanation.

---

## Notes

This is a working prototype, not a commercial product. The tracking quality depends on:

* lighting
* ESP32-CAM stream quality
* camera latency
* servo power
* mechanical alignment
* target visibility
* YOLO model quality
* tuning values in the Python and Arduino code

Some values may need to be adjusted for different hardware or 3D-printed parts.

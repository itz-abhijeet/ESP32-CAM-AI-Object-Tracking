#include <Wire.h>
#include <math.h>

#define PCA9685_ADDR 0x40

#define PAN_CH 0
#define TILT_CH 1

/*
  Servo controller for ESP32-CAM AI object tracking project

  Hardware:
  - Second ESP32-WROOM-32 DevKit
  - PCA9685 servo driver
  - Pan servo on PCA9685 channel 0
  - Tilt servo on PCA9685 channel 1
  - I2C: GPIO21 = SDA, GPIO22 = SCL

  Commands from Python:
  - C = center servos
  - D,panDelta,tiltDelta = move pan/tilt servos
*/

// SERVO CENTER

int panCenter = 300;
int tiltCenter = 410;

float panPos = panCenter;
float tiltPos = tiltCenter;

float targetPan = panCenter;
float targetTilt = tiltCenter;


// CENTER ANIMATION STATE

bool centeringMode = false;

float centerStartPan = 0;
float centerStartTilt = 0;

unsigned long centerStartTime = 0;

// Higher = slower
unsigned long centerDurationMs = 1150;

int centerWriteDeadband = 1;

// SERVO LIMITS

// PAN = left/right
int panMin = 180;
int panMax = 430;

// TILT = up/down
int tiltMin = 230;
int tiltMax = 470;

int PAN_DIR = 1;
int TILT_DIR = 1;

// TRACKING SETTINGS

unsigned long servoUpdateMs = 6;
unsigned long lastServoUpdate = 0;

int panMaxTargetLead = 45;
int tiltMaxTargetLead = 40;

unsigned long commandTimeoutMs = 500;
unsigned long lastCommandTime = 0;

float arriveThreshold = 2;

// PAN tuning
float panMinStep = 0.8;
float panMaxStep = 6.2;
float panGain = 0.15;

// TILT tuning
float tiltMinStep = 0.5;
float tiltMaxStep = 4.0;
float tiltGain = 0.09;

int tinyPanDeltaIgnore = 1;
int tinyTiltDeltaIgnore = 0;

int panWriteDeadband = 2;
int tiltWriteDeadband = 2;

int lastPanPWM = -9999;
int lastTiltPWM = -9999;

// SERIAL LINE BUFFER

char serialBuffer[48];
int serialIndex = 0;

// PCA9685 LOW LEVEL FUNCTIONS

void writeRegister(byte reg, byte value) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void setPWM(byte channel, int on, int off) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(0x06 + 4 * channel);

  Wire.write(on & 0xFF);
  Wire.write(on >> 8);

  Wire.write(off & 0xFF);
  Wire.write(off >> 8);

  Wire.endTransmission();
}

void setupPCA9685() {
  writeRegister(0x00, 0x10);  // sleep
  writeRegister(0xFE, 121);   // about 50 Hz for servos
  writeRegister(0x00, 0x00);  // wake
  delay(10);
  writeRegister(0x00, 0xA1);  // auto-increment
}

void moveServo(byte ch, int pwm) {
  setPWM(ch, 0, pwm);
}

void moveServoIfChanged(byte ch, int pwm, int &lastPWM, int deadband) {
  if (lastPWM == -9999 || abs(pwm - lastPWM) >= deadband) {
    moveServo(ch, pwm);
    lastPWM = pwm;
  }
}

// HELPER FUNCTIONS

float absFloat(float value) {
  if (value < 0) return -value;
  return value;
}

float clampFloat(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

float easeInOutCos(float t) {
  t = clampFloat(t, 0.0, 1.0);
  return 0.5 - 0.5 * cos(t * 3.14159265);
}

float adaptiveStep(float diff, float minStep, float maxStep, float gain) {
  float absDiff = absFloat(diff);

  if (absDiff <= arriveThreshold) {
    return absDiff;
  }

  float step = minStep + absDiff * gain;

  if (step > maxStep) step = maxStep;
  if (step < minStep) step = minStep;

  return step;
}

void freezeTargetsToCurrent() {
  centeringMode = false;
  targetPan = panPos;
  targetTilt = tiltPos;
}

void centerServos() {
  centerStartPan = panPos;
  centerStartTilt = tiltPos;
  centerStartTime = millis();

  targetPan = panCenter;
  targetTilt = tiltCenter;

  centeringMode = true;
}

void applyTargetLeadLimit() {
  targetPan = clampFloat(
    targetPan,
    panPos - panMaxTargetLead,
    panPos + panMaxTargetLead
  );

  targetTilt = clampFloat(
    targetTilt,
    tiltPos - tiltMaxTargetLead,
    tiltPos + tiltMaxTargetLead
  );

  targetPan = clampFloat(targetPan, panMin, panMax);
  targetTilt = clampFloat(targetTilt, tiltMin, tiltMax);
}

// SERVO UPDATE

void updateSmoothServos() {
  unsigned long now = millis();

  if (now - lastServoUpdate < servoUpdateMs) {
    return;
  }

  lastServoUpdate = now;

  if (centeringMode) {
    float t = (now - centerStartTime) / (float)centerDurationMs;

    if (t >= 1.0) {
      panPos = panCenter;
      tiltPos = tiltCenter;

      targetPan = panCenter;
      targetTilt = tiltCenter;

      centeringMode = false;
    } else {
      float e = easeInOutCos(t);

      panPos = centerStartPan + (panCenter - centerStartPan) * e;
      tiltPos = centerStartTilt + (tiltCenter - centerStartTilt) * e;
    }

    panPos = clampFloat(panPos, panMin, panMax);
    tiltPos = clampFloat(tiltPos, tiltMin, tiltMax);

    int panPWM = (int)round(panPos);
    int tiltPWM = (int)round(tiltPos);

    moveServoIfChanged(PAN_CH, panPWM, lastPanPWM, centerWriteDeadband);
    moveServoIfChanged(TILT_CH, tiltPWM, lastTiltPWM, centerWriteDeadband);

    return;
  }

  if (now - lastCommandTime > commandTimeoutMs) {
    freezeTargetsToCurrent();
  }

  applyTargetLeadLimit();

  float panDiff = targetPan - panPos;
  float tiltDiff = targetTilt - tiltPos;

  // PAN / LEFT-RIGHT
  if (absFloat(panDiff) <= arriveThreshold) {
    panPos = targetPan;
  } else {
    float step = adaptiveStep(
      panDiff,
      panMinStep,
      panMaxStep,
      panGain
    );

    if (panDiff > 0) {
      panPos += step;
      if (panPos > targetPan) panPos = targetPan;
    } else {
      panPos -= step;
      if (panPos < targetPan) panPos = targetPan;
    }
  }

  // TILT / UP-DOWN
  if (absFloat(tiltDiff) <= arriveThreshold) {
    tiltPos = targetTilt;
  } else {
    float step = adaptiveStep(
      tiltDiff,
      tiltMinStep,
      tiltMaxStep,
      tiltGain
    );

    if (tiltDiff > 0) {
      tiltPos += step;
      if (tiltPos > targetTilt) tiltPos = targetTilt;
    } else {
      tiltPos -= step;
      if (tiltPos < targetTilt) tiltPos = targetTilt;
    }
  }

  panPos = clampFloat(panPos, panMin, panMax);
  tiltPos = clampFloat(tiltPos, tiltMin, tiltMax);

  int panPWM = (int)round(panPos);
  int tiltPWM = (int)round(tiltPos);

  moveServoIfChanged(PAN_CH, panPWM, lastPanPWM, panWriteDeadband);
  moveServoIfChanged(TILT_CH, tiltPWM, lastTiltPWM, tiltWriteDeadband);
}

// COMMAND HANDLING

void handleCommand(char *cmd) {
  if (cmd[0] == 'C' || cmd[0] == 'c') {
    centerServos();
    lastCommandTime = millis();
    Serial.println("CENTER");
    return;
  }

  int panDelta = 0;
  int tiltDelta = 0;

  if (sscanf(cmd, "D,%d,%d", &panDelta, &tiltDelta) == 2 ||
      sscanf(cmd, "d,%d,%d", &panDelta, &tiltDelta) == 2) {

    lastCommandTime = millis();


    centeringMode = false;

    if (panDelta == 0 && tiltDelta == 0) {
      freezeTargetsToCurrent();
      return;
    }


    if (abs(panDelta) <= tinyPanDeltaIgnore) {
      panDelta = 0;
    }


    if (abs(tiltDelta) <= tinyTiltDeltaIgnore) {
      tiltDelta = 0;
    }

    if (panDelta == 0 && tiltDelta == 0) {
      return;
    }

    panDelta *= PAN_DIR;
    tiltDelta *= TILT_DIR;

    targetPan += panDelta;
    targetTilt += tiltDelta;

    targetPan = clampFloat(targetPan, panMin, panMax);
    targetTilt = clampFloat(targetTilt, tiltMin, tiltMax);

    applyTargetLeadLimit();
  }
}

void readSerialNonBlocking() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialIndex > 0) {
        serialBuffer[serialIndex] = '\0';
        handleCommand(serialBuffer);
        serialIndex = 0;
      }
    } else {
      if (serialIndex < sizeof(serialBuffer) - 1) {
        serialBuffer[serialIndex++] = c;
      } else {
        serialIndex = 0;
      }
    }
  }
}


// SETUP / LOOP


void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  setupPCA9685();

  int panPWM = (int)round(panPos);
  int tiltPWM = (int)round(tiltPos);

  moveServo(PAN_CH, panPWM);
  moveServo(TILT_CH, tiltPWM);

  lastPanPWM = panPWM;
  lastTiltPWM = tiltPWM;

  lastCommandTime = millis();

  Serial.println("READY PAN FAST / SMOOTH CENTER");
}

void loop() {
  readSerialNonBlocking();
  updateSmoothServos();
}
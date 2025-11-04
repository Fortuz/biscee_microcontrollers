#include <Servo.h>

// ------ SERVO SETUP ------
Servo servo1, servo2, servo3;
int pin_blue = 9;
int pin_green = 10;
int pin_orange = 11;

const int MIN_POS = 20;
const int MAX_POS = 150;

// Position memory
float currentPos[3] = {90, 90, 90};
float startPos[3]   = {90, 90, 90};
float targetPos[3]  = {90, 90, 90};

// Timing for interpolation
unsigned long moveStart = 0;
unsigned long moveDuration = 1000;   // ms

// Safety rate limit (deg/update)
const float MAX_STEP = 2.0;
const unsigned long UPDATE_INTERVAL = 20;
unsigned long lastUpdate = 0;

const byte START_BYTE = 0xAA;

void setup() {
  Serial.begin(115200);
  servo1.attach(pin_blue);
  servo2.attach(pin_orange);
  servo3.attach(pin_green);

  servo1.write(currentPos[0]);
  servo2.write(currentPos[1]);
  servo3.write(currentPos[2]);
}

void loop() {
  handleIncoming();
  updateMotion();
  sendFeedback();
}

// ------ READ SERIAL INPUT ------
void handleIncoming() {
  static byte buf[7];
  static byte idx = 0;

  while (Serial.available()) {
    byte b = Serial.read();

    if (idx == 0 && b != START_BYTE) continue;

    buf[idx++] = b;

    if (idx == 7) {
      byte a1 = buf[1];
      byte a2 = buf[2];
      byte a3 = buf[3];
      byte tL = buf[4];
      byte tH = buf[5];

      byte cs = buf[6];
      if ((a1 ^ a2 ^ a3 ^ tL ^ tH) == cs) {

        // Update targets
        targetPos[0] = constrain(a1, MIN_POS, MAX_POS);
        targetPos[1] = constrain(a2, MIN_POS, MAX_POS);
        targetPos[2] = constrain(a3, MIN_POS, MAX_POS);

        // Store starting positions for interpolation
        for (int i=0; i<3; i++) startPos[i] = currentPos[i];

        // Extract duration (uint16, little-endian)
        moveDuration = ((uint16_t)tH << 8) | tL;
        moveDuration = max(moveDuration, (uint16_t)20); // avoid divide-by-zero

        moveStart = millis();
      }
      idx = 0;
    }
  }
}

// ------ INTERPOLATED MOTION ------
void updateMotion() {
  unsigned long now = millis();
  if (now - lastUpdate < UPDATE_INTERVAL) return;
  lastUpdate = now;

  float ratio = float(now - moveStart) / moveDuration;
  if (ratio > 1.0) ratio = 1.0;

  for (int i=0; i<3; i++) {
    float desired = startPos[i] + ratio * (targetPos[i] - startPos[i]);

    // Rate-limiting safety
    float diff = desired - currentPos[i];
    if (abs(diff) > MAX_STEP)
      currentPos[i] += (diff > 0 ? MAX_STEP : -MAX_STEP);
    else
      currentPos[i] = desired;

    // Send to servo
    servo1.write(currentPos[0]);
    servo2.write(currentPos[1]);
    servo3.write(currentPos[2]);
  }
}

// ------ FEEDBACK PACKET ------
void sendFeedback() {
  static unsigned long lastSend = 0;
  if (millis() - lastSend < 100) return;
  lastSend = millis();

  byte c1 = (byte)currentPos[0];
  byte c2 = (byte)currentPos[1];
  byte c3 = (byte)currentPos[2];
  byte checksum = c1 ^ c2 ^ c3;

  Serial.write(0xAB);
  Serial.write(c1);
  Serial.write(c2);
  Serial.write(c3);
  Serial.write(checksum);
}

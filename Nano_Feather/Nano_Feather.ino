#include <Servo.h>

Servo servo;

class SecondOrderDynamics {
  float xp, y, yd;
  float k1, k2, k3;

public:
  SecondOrderDynamics(float f, float z, float r, float x0) {
    float PI = 3.14159265359;
    k1 = z / (PI * f);
    k2 = 1.0 / ((2.0 * PI * f) * (2.0 * PI * f));
    k3 = r * z / (2.0 * PI * f);
    xp = x0;
    y = x0;
    yd = 0;
  }

  float update(float T, float x, float xd = NAN) {
    if (isnan(xd)) {
      xd = (x - xp) / T;  // estimate velocity if not given
      xp = x;
    }
    float k2_stable = max(k2, 1.1f * (T * T / 4.0f + T * k1 / 2.0f));
    y = y + T * yd;
    yd = yd + T * ((x + k3 * xd - y - k1 * yd) / k2_stable);
    return y;
  }
};

SecondOrderDynamics motion(1.0, 0.5, 1.0, 90.0); // frequency, damping, response, initial angle

float targetAngle = 90.0;
unsigned long lastTime;

void setup() {
  servo.attach(9);  // servo on pin 9
  servo.write(90);
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  float T = (now - lastTime) / 1000.0;
  lastTime = now;

  // Example pattern: move between 45° and 135° every 3 seconds
  float time = millis() / 1000.0;
  targetAngle = 90 + 45 * sin(time * 2 * 3.14159 / 3.0);

  float smoothAngle = motion.update(T, targetAngle);
  servo.write(smoothAngle);

  delay(15);
}

#include <Servo.h>

Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo gripperServo;

// Link lengths (cm)
float L1 = 8.5;
float L2 = 8.5;

// Servo offsets
int baseOffset = 90;
int shoulderOffset = 90;
int elbowOffset = 90;

int gripperOpen = 40;
int gripperClose = 130;

void setup() {
  Serial.begin(9600);

  baseServo.attach(3);
  shoulderServo.attach(5);
  elbowServo.attach(6);
  gripperServo.attach(9);

  baseServo.write(90);
  shoulderServo.write(90);
  elbowServo.write(90);
  gripperServo.write(gripperOpen);

  delay(2000);
  Serial.println("READY");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("MOVE")) {
      float x, y, z;
      sscanf(cmd.c_str(), "MOVE %f %f %f", &x, &y, &z);
      moveTo(x, y, z);
    }

    if (cmd == "GRIP OPEN") openGripper();
    if (cmd == "GRIP CLOSE") closeGripper();
  }
}

void moveTo(float x, float y, float z) {
  float r = sqrt(x*x + y*y);
  if (r < 1.0) r = 1.0;

  float baseAngle = atan2(y, x);
  int baseDeg = degrees(baseAngle) + baseOffset;

  float d = sqrt(r*r + z*z);
  if (d > (L1 + L2) || d < abs(L1 - L2)) {
    Serial.println("OUT_OF_REACH");
    return;
  }

  float cosElbow = (d*d - L1*L1 - L2*L2) / (2 * L1 * L2);
  cosElbow = constrain(cosElbow, -1.0, 1.0);

  float elbowAngle = acos(cosElbow);
  float shoulderAngle =
    atan2(z, r) -
    atan2(L2 * sin(elbowAngle),
          L1 + L2 * cos(elbowAngle));

  int shoulderDeg = degrees(shoulderAngle) + shoulderOffset;
  int elbowDeg = degrees(elbowAngle) + elbowOffset;

  baseServo.write(constrain(baseDeg, 0, 180));
  shoulderServo.write(constrain(shoulderDeg, 20, 160));
  elbowServo.write(constrain(elbowDeg, 10, 170));
}

void openGripper() {
  gripperServo.write(gripperOpen);
}

void closeGripper() {
  gripperServo.write(gripperClose);
}

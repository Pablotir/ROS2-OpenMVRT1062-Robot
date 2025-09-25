#include "Wire.h"
#include "HiTechnicController.h"
#include "HiTechnicServoController.h"
#include "HiTechnicServo.h"

using DaisyChainPosition = HiTechnicController::DaisyChainPosition;
using ServoPort          = HiTechnicServoController::ServoPort;

// --- Controller (3rd in the daisy chain) ---
HiTechnicServoController sc(DaisyChainPosition::THIRD);

// --- Servos on ports 1–6 ---
HiTechnicServo s1(&sc, ServoPort::PORT_1);
HiTechnicServo s2(&sc, ServoPort::PORT_2);
HiTechnicServo s3(&sc, ServoPort::PORT_3);
HiTechnicServo s4(&sc, ServoPort::PORT_4);
HiTechnicServo s5(&sc, ServoPort::PORT_5);
HiTechnicServo s6(&sc, ServoPort::PORT_6);

float degreesToNorm(int deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  return (float)deg / 180.0f;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();
  Wire.setClock(37390L);

  Serial.println("Servo command interface ready.");
  Serial.println("Use: S<port>,<deg>  (e.g. S2,120)");

  sc.enablePwm(true);

  // Default all servos to 90°
  s1.setPosition(degreesToNorm(90));
  s2.setPosition(degreesToNorm(90));
  s3.setPosition(degreesToNorm(90));
  s4.setPosition(degreesToNorm(90));
  s5.setPosition(degreesToNorm(90));
  s6.setPosition(degreesToNorm(90));
}

void setServo(int port, int deg) {
  float pos = degreesToNorm(deg);
  switch (port) {
    case 1: s1.setPosition(pos); break;
    case 2: s2.setPosition(pos); break;
    case 3: s3.setPosition(pos); break;
    case 4: s4.setPosition(pos); break;
    case 5: s5.setPosition(pos); break;
    case 6: s6.setPosition(pos); break;
    default:
      Serial.print("Invalid servo port: "); Serial.println(port);
      return;
  }
  Serial.print("Set servo "); Serial.print(port);
  Serial.print(" to "); Serial.print(deg); Serial.println("°");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.startsWith("S")) {
      int commaIndex = line.indexOf(',');
      if (commaIndex > 1) {
        int port = line.substring(1, commaIndex).toInt();
        int deg  = line.substring(commaIndex + 1).toInt();
        setServo(port, deg);
      }
    }
  }
}

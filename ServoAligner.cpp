#include "Wire.h"
#include "HiTechnicController.h"
#include "HiTechnicServoController.h"
#include "HiTechnicServo.h"

using DaisyChainPosition = HiTechnicController::DaisyChainPosition;
using ServoPort          = HiTechnicServoController::ServoPort;

// --- Flags (enable/disable individual servos) ---
bool ENABLE_S1 = true;
bool ENABLE_S2 = true;
bool ENABLE_S3 = true;
bool ENABLE_S4 = true;
bool ENABLE_S5 = true;
bool ENABLE_S6 = true;

// --- Controller (3rd in the daisy chain) ---
HiTechnicServoController sc(DaisyChainPosition::THIRD);

// --- Servos ---
HiTechnicServo s1(&sc, ServoPort::PORT_1);
HiTechnicServo s2(&sc, ServoPort::PORT_2);
HiTechnicServo s3(&sc, ServoPort::PORT_3);
HiTechnicServo s4(&sc, ServoPort::PORT_4);
HiTechnicServo s5(&sc, ServoPort::PORT_5);
HiTechnicServo s6(&sc, ServoPort::PORT_6);

// --- Helper: map degrees [0–180] to normalized [0.0–1.0] ---
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

  Serial.println("Starting HiTechnic Servo test...");
  sc.enablePwm(true);

  if (ENABLE_S1) { s1.setPosition(degreesToNorm(90)); Serial.println("Servo 1 -> 90°"); }
  if (ENABLE_S2) { s2.setPosition(degreesToNorm(90)); Serial.println("Servo 2 -> 90°"); }
  if (ENABLE_S3) { s3.setPosition(degreesToNorm(90)); Serial.println("Servo 3 -> 90°"); }
  if (ENABLE_S4) { s4.setPosition(degreesToNorm(90)); Serial.println("Servo 4 -> 90°"); }
  if (ENABLE_S5) { s5.setPosition(degreesToNorm(90)); Serial.println("Servo 5 -> 90°"); }
  if (ENABLE_S6) { s6.setPosition(degreesToNorm(90)); Serial.println("Servo 6 -> 90°"); }
}

void loop() {
  // Nothing else — each enabled servo holds 90°
}

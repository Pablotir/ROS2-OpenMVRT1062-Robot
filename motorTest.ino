#include "Wire.h"
#include "HiTechnicController.h"
#include "HiTechnicDcMotorController.h"
#include "HiTechnicMotor.h"

using DaisyChainPosition = HiTechnicController::DaisyChainPosition;
using MotorPort          = HiTechnicDcMotorController::MotorPort;
using RunMode            = HiTechnicDcMotorController::RunMode;
using ZeroPowerBehavior  = HiTechnicDcMotorController::ZeroPowerBehavior;
using Direction          = HiTechnicMotor::Direction;

#define TICKS_PER_REV 1440
#define WHEEL_DIAMETER_IN 4.0
#define DEFAULT_PWR 0.30f

// --- Enable/disable motors here ---
bool ENABLE_BL = true;  // Back Left
bool ENABLE_BR = true;  // Back Right
bool ENABLE_FL = false; // Front Left
bool ENABLE_FR = false; // Front Right

// --- Controllers ---
HiTechnicDcMotorController mc1(DaisyChainPosition::FIRST);   // left side
HiTechnicDcMotorController mc2(DaisyChainPosition::SECOND);  // right side

// --- Motors ---
HiTechnicMotor mBL(&mc1, MotorPort::PORT_1);  // Back Left (invert target)
HiTechnicMotor mBR(&mc2, MotorPort::PORT_2);  // Back Right (normal target)
HiTechnicMotor mFL(&mc1, MotorPort::PORT_2);  // Front Left (normal target)
HiTechnicMotor mFR(&mc2, MotorPort::PORT_1);  // Front Right (invert target)

long inchesToTicks(float inches) {
  float circumference = WHEEL_DIAMETER_IN * 3.14159f;
  float revs = inches / circumference;
  return (long)(revs * TICKS_PER_REV);
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Starting motor test with flags...");

  Wire.begin();
  Wire.setClock(37390L);

  // Reset encoders only for enabled motors
  if (ENABLE_BL) mBL.setMode(RunMode::STOP_AND_RESET_ENCODER);
  if (ENABLE_BR) mBR.setMode(RunMode::STOP_AND_RESET_ENCODER);
  if (ENABLE_FL) mFL.setMode(RunMode::STOP_AND_RESET_ENCODER);
  if (ENABLE_FR) mFR.setMode(RunMode::STOP_AND_RESET_ENCODER);
  delay(50);

  // Brake mode
  if (ENABLE_BL) mBL.setZeroPowerBehavior(ZeroPowerBehavior::BRAKE);
  if (ENABLE_BR) mBR.setZeroPowerBehavior(ZeroPowerBehavior::BRAKE);
  if (ENABLE_FL) mFL.setZeroPowerBehavior(ZeroPowerBehavior::BRAKE);
  if (ENABLE_FR) mFR.setZeroPowerBehavior(ZeroPowerBehavior::BRAKE);

  // Direction (ignored in RUN_TO_POSITION but set for consistency)
  if (ENABLE_BL) mBL.setDirection(Direction::FORWARD);
  if (ENABLE_BR) mBR.setDirection(Direction::FORWARD);
  if (ENABLE_FL) mFL.setDirection(Direction::FORWARD);
  if (ENABLE_FR) mFR.setDirection(Direction::FORWARD);

  mc1.setTimeoutEnabled(true);
  mc2.setTimeoutEnabled(true);

  long ticks = inchesToTicks(2.0f);

  Serial.print("Target ticks: ");
  Serial.println(ticks);

  // Assign targets only for enabled motors
  if (ENABLE_BL) { mBL.setTargetPosition(-ticks); mBL.setMode(RunMode::RUN_TO_POSITION); mBL.setPower(DEFAULT_PWR); }
  if (ENABLE_BR) { mBR.setTargetPosition(+ticks); mBR.setMode(RunMode::RUN_TO_POSITION); mBR.setPower(DEFAULT_PWR); }
  if (ENABLE_FL) { mFL.setTargetPosition(+ticks); mFL.setMode(RunMode::RUN_TO_POSITION); mFL.setPower(DEFAULT_PWR); }
  if (ENABLE_FR) { mFR.setTargetPosition(-ticks); mFR.setMode(RunMode::RUN_TO_POSITION); mFR.setPower(DEFAULT_PWR); }
}

void loop() {
  bool busy = false;

  if (ENABLE_BL) busy |= mBL.isBusy();
  if (ENABLE_BR) busy |= mBR.isBusy();
  if (ENABLE_FL) busy |= mFL.isBusy();
  if (ENABLE_FR) busy |= mFR.isBusy();

  if (ENABLE_BL) { Serial.print("BL: "); Serial.print(mBL.getCurrentPosition()); Serial.print("/"); Serial.print(mBL.getTargetPosition()); Serial.print(" | "); }
  if (ENABLE_BR) { Serial.print("BR: "); Serial.print(mBR.getCurrentPosition()); Serial.print("/"); Serial.print(mBR.getTargetPosition()); Serial.print(" | "); }
  if (ENABLE_FL) { Serial.print("FL: "); Serial.print(mFL.getCurrentPosition()); Serial.print("/"); Serial.print(mFL.getTargetPosition()); Serial.print(" | "); }
  if (ENABLE_FR) { Serial.print("FR: "); Serial.print(mFR.getCurrentPosition()); Serial.print("/"); Serial.print(mFR.getTargetPosition()); }
  Serial.println();

  if (!busy) {
    if (ENABLE_BL) mBL.setPower(0);
    if (ENABLE_BR) mBR.setPower(0);
    if (ENABLE_FL) mFL.setPower(0);
    if (ENABLE_FR) mFR.setPower(0);

    Serial.println("Movement complete. Holding with BRAKE.");
    while(1); // stop here
  }

  delay(100);
}

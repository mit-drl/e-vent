#include "Utilities.h"


namespace utils {


/// Pulse ///

Pulse::Pulse(const unsigned long& period, const float& duty, const bool& random_offset):
    period_(period),
    on_duration_(duty * period),
    offset_(random_offset ? random(period) : 0) {}

bool Pulse::read() {
  return (millis() - offset_) % period_ < on_duration_;
}


float ticks2volume(const float& vol_ticks) {
  return COEFFS.a * sqr(vol_ticks) + COEFFS.b * vol_ticks + COEFFS.c;
}


float volume2ticks(const float& vol_ml) {
  return (-COEFFS.b + sqrt(sqr(COEFFS.b) -4 * COEFFS.a * (COEFFS.c - vol_ml))) / (2 * COEFFS.a);
}


int readVolume() {
  return map(analogRead(VOL_PIN), 0, ANALOG_PIN_MAX, VOL_MIN, VOL_MAX);
}

int readBpm() {
  return map(analogRead(BPM_PIN), 0, ANALOG_PIN_MAX, BPM_MIN, BPM_MAX);
}

float readIeRatio() {
  // Carry one decimal place
  return map(analogRead(IE_PIN), 0, ANALOG_PIN_MAX, IE_MIN * 10, IE_MAX * 10) / 10.0;
}

float readTriggerSens() {
  // Carry two decimal places
  return map(analogRead(AC_PIN), 0, ANALOG_PIN_MAX, 0, AC_MAX * 100) / 100.0;
}


bool readEncoder(const RoboClaw& roboclaw, int& motorPosition) {
  uint8_t robot_status;
  bool valid;
  motorPosition = roboclaw.ReadEncM1(ROBOCLAW_ADDR, &robot_status, &valid);
  return valid;
}


void goToPosition(const RoboClaw& roboclaw, const int& pos, const int& vel) {
  int motor_position;
  const bool valid = readEncoder(roboclaw, motor_position);

  const int accel = 10000;
  const int deccel = 10000;
  
  if (valid) {
    roboclaw.SpeedAccelDeccelPositionM1(ROBOCLAW_ADDR, accel, vel, deccel, pos, 1);
    if (DEBUG) {
      Serial.print("CmdVel: ");  // TODO remove
      Serial.print(vel);
      Serial.print("\tCmdDiff: ");
      Serial.println(pos);
    }
  }
  else{
    if (DEBUG) {
      Serial.println("encoder not valid; goToPosition command not sent");
    }
    // ELSE THROW AN ALARM
  }
}


bool readMotorCurrent(const RoboClaw& roboclaw, int& motorCurrent) {
  int noSecondMotor;
  const bool valid = roboclaw.ReadCurrents(ROBOCLAW_ADDR, motorCurrent, noSecondMotor);
  return valid;
}


}  // namespace utils

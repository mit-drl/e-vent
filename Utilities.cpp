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


float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float ticks2volume(const float& vol_ticks) {
  return COEFFS.a * sqr(vol_ticks) + COEFFS.b * vol_ticks + COEFFS.c;
}


float volume2ticks(const float& vol_ml) {
  return (-COEFFS.b + sqrt(sqr(COEFFS.b) -4 * COEFFS.a * (COEFFS.c - vol_ml))) / (2 * COEFFS.a);
}


float readVolume() {
  return map(analogRead(VOL_PIN), 0, ANALOG_PIN_MAX, VOL_MIN, VOL_MAX);
}

float readBpm() {
  return map(analogRead(BPM_PIN), 0, ANALOG_PIN_MAX, BPM_MIN, BPM_MAX);
}

float readIeRatio() {
  return map(analogRead(IE_PIN), 0, ANALOG_PIN_MAX, IE_MIN, IE_MAX);
}

float readTriggerSens() {
  return map(analogRead(AC_PIN), 0, ANALOG_PIN_MAX, AC_MIN - 0.1, AC_MAX);
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

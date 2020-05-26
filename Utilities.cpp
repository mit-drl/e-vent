/**
 * MIT Emergency Ventilator Controller
 * 
 * MIT License:
 * 
 * Copyright (c) 2020 MIT
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * Utilities.cpp
 */

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
  return (-COEFFS.b + sqrt(sqr(COEFFS.b) - 4 * COEFFS.a * (COEFFS.c - vol_ml))) / (2 * COEFFS.a);
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

float readAc() {
  return map(analogRead(AC_PIN), 0, ANALOG_PIN_MAX, AC_MIN - AC_RES, AC_MAX);
}

bool readEncoder(const RoboClaw& roboclaw, int& motorPosition) {
  uint8_t robot_status;
  bool valid;
  motorPosition = roboclaw.ReadEncM1(ROBOCLAW_ADDR, &robot_status, &valid);
  return valid;
}

void goToPosition(const RoboClaw& roboclaw, const long& pos, const long& vel, const long& acc) {
    roboclaw.SpeedAccelDeccelPositionM1(ROBOCLAW_ADDR, acc, vel, acc, pos, 1); 
}

void goToPositionByDur(const RoboClaw& roboclaw, const long& goal_pos, const long& cur_pos, const float& dur) {
  if (dur <= 0) return; // Can't move in negative time

  const long dist = abs(goal_pos - cur_pos);
  long vel = round(2*dist/dur); // Try bang-bang control
  long acc = round(2*vel/dur); // Constant acc in and out
  if (vel > VEL_MAX) {
    // Must use trapezoidal velocity profile to clip at VEL_MAX
    vel = VEL_MAX;
    const float acc_dur = dur - dist/vel;
    acc = acc_dur > 0 ? round(vel/acc_dur) : ACC_MAX;
    acc = min(ACC_MAX, acc);
  }

  goToPosition(roboclaw, goal_pos, vel, acc);
}

bool readMotorCurrent(const RoboClaw& roboclaw, int& motorCurrent) {
  int noSecondMotor;
  const bool valid = roboclaw.ReadCurrents(ROBOCLAW_ADDR, motorCurrent, noSecondMotor);
  return valid;
}


}  // namespace utils

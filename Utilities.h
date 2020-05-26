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
 * Utilities.h
 * Defines a variety of functions used in `e-vent.ino` or other files.
 */

#ifndef Utilities_h
#define Utilities_h

#include "Arduino.h"
#include "src/thirdparty/RoboClaw/RoboClaw.h"

#include "cpp_utils.h"

#include "Constants.h"


namespace utils {


/**
 * Pulse
 * Generates an ON/OFF signal with given period and duty.
 */
class Pulse {
public:
  Pulse(const unsigned long& period, const float& duty, const bool& random_offset = true);

  // Read current ON/OFF value
  bool read();

private:
  const unsigned long period_;
  const unsigned long on_duration_;
  const unsigned long offset_;
};


// Define map for floats
float map(float x, float in_min, float in_max, float out_min, float out_max);

// Converts motor position in ticks to volume in mL
float ticks2volume(const float& vol_ticks);

// Converts volume in mL to motor position in ticks
float volume2ticks(const float& vol_ml);

// Returns the current time in seconds
inline float now() { return millis()*1e-3; }

// Home switch
inline bool homeSwitchPressed() { return digitalRead(HOME_PIN) == LOW; }

/// Pots ///
float readVolume();       // Reads set volume (in mL) from the volume pot
float readBpm();          // Reads set bpm from the bpm pot
float readIeRatio();      // Reads set IE ratio from the IE pot
float readAc();           // Reads set AC mode trigger sensitivity from the AC pot

/// Motor ///
// Read the encoder and return whether the reading is valid
bool readEncoder(const RoboClaw& roboclaw, int& motorPosition);

// Go to a desired position at the given speed
void goToPosition(const RoboClaw& roboclaw, const long& pos, const long& vel, const long& acc);

// Go to a desired position over the specified duration
void goToPositionByDur(const RoboClaw& roboclaw, const long& goal_pos, const long& cur_pos, const float& dur);

// Read the motor current and return whether the reading is valid
bool readMotorCurrent(const RoboClaw& roboclaw, int& motorCurrent);


}  // namespace utils


#endif


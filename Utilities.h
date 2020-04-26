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
float readTriggerSens();  // Reads set trigger sensitivity from the trigger pot

/// Motor ///
// Read the encoder and return whether the reading is valid
bool readEncoder(const RoboClaw& roboclaw, int& motorPosition);

// Go to a desired position at the given speed
void goToPosition(const RoboClaw& roboclaw, const int& pos, const int& vel);

// Read the motor current and return whether the reading is valid
bool readMotorCurrent(const RoboClaw& roboclaw, int& motorCurrent);


}  // namespace utils


#endif


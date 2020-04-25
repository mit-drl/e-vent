#ifndef Utilities_h
#define Utilities_h

#include "Arduino.h"

#include "Constants.h"


namespace utils {


/**
 * Pulse
 * Generates an ON/OFF signal with given period and duty.
 */
class Pulse {
public:
  Pulse(const unsigned long& period, const float& duty, const bool& random_offset = true):
      period_(period),
      on_duration_(duty * period),
      offset_(random_offset ? random(period) : 0) {}

  // Read current ON/OFF value
  bool read() {
    return (millis() - offset_) % period_ < on_duration_;
  }

private:
  const unsigned long period_;
  const unsigned long on_duration_;
  const unsigned long offset_;
};


}  // namespace utils


#endif


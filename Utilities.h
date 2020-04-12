#ifndef Utilities_h
#define Utilities_h

#include "Arduino.h"


namespace utilities {


/**
 * DebouncedButton
 * Represents a pullup button that filters out unintended LOW readings.
 */
class DebouncedButton {

  const unsigned long kDebounceDelay = 100;

public:
  DebouncedButton(const int& pin);

  // Setup during arduino setup()
  void begin();

  // Check if button is low
  bool is_LOW();

private:
  int pin_;
  unsigned long last_low_time_ = 0;
};


/**
 * Pulse
 * Generates an ON/OFF signal with given period and duty.
 */
class Pulse {
private:
  Pulse(const unsigned long& period, const float& duty, const bool& random_offset = true):
      period_(period),
      on_duration_(duty * period),
      offset_(random_offset ? random(period) : 0) {}

  // Read current ON/OFF value
  bool read() {
    return (millis() - offset_) % period_ < on_duration_;
  }

public:
  const unsigned long period_;
  const unsigned long on_duration_;
  const unsigned long offset_;
};


}  // namespace utilities


#endif


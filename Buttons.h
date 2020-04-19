#ifndef Buttons_h
#define Buttons_h

#include "Arduino.h"


namespace buttons {


/**
 * PressHoldButton
 * Button abstraction capable of detecting if a given button has been held
 * pressed for a given ammount of time.
 */
class PressHoldButton {

  const unsigned long kDebounceDelay = 100;

public:
  PressHoldButton(const int& pin, const unsigned long& hold_duration):
    pin_(pin),
    hold_duration_(hold_duration) {}

  // Setup during arduino setup()
  inline void begin() { pinMode(pin_, INPUT_PULLUP); }

  // Update during arduino loop()
  void update();

  // Check if button was just held for hold_duration ms.
  inline bool wasHeld() { return current_hold_time_ > hold_duration_; }

private:
  int pin_;
  unsigned long hold_duration_;
  unsigned long last_low_time_ = 0;
  unsigned long current_hold_time_ = 0;
};


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


}  // namespace buttons


#endif


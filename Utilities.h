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


}  // namespace utilities


#endif


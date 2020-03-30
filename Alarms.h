#ifndef Alarms_h
#define Alarms_h

#include "Arduino.h"

#include "Display.h"


namespace alarms {

static const unsigned long SNOOZE_TIME = 1 * 60 * 1000;  // in milliseconds


using display::Display;


class Alarm {
public:
  // Construct from beeper pin and display
  Alarm(const int& beeper_pin, Display* displ);

  // Update alarm: meant to be called every loop
  void update();

  // Sound alarm and show message
  void loud(const String& alarm_message);

  // Silent alarm: display message only
  void silent(const String& alarm_message);

  // Clear alarm: stop beeper and hide message
  void clear();

  // Snooze alarm: stop beeper for SNOOZE_TIME milliseconds
  void snooze(); 

private:
  const int beeper_pin_;
  Display* displ_;
  String text_ = "";
  bool beep_ = false;
  unsigned long snooze_time_ = 0;
  bool snoozed_ = false;

  inline void beeperON() { digitalWrite(beeper_pin_, HIGH); }

  inline void beeperOFF() { digitalWrite(beeper_pin_, LOW); }
};


} // namespace alarms


#endif

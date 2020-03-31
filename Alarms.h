#ifndef Alarms_h
#define Alarms_h

#include "Arduino.h"

#include "Display.h"


namespace alarms {

static const unsigned long SNOOZE_TIME = 2 * 60UL * 1000;  // in milliseconds


using display::Display;


/// Alarm ///

class Alarm {
public:
  // Construct from pins and display
  Alarm(Display* displ);

  // Setup 
  void begin();

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
  String text_ = "";
  bool beep_ = false;
  unsigned long snooze_time_ = 0;
  bool snoozed_ = false;

  inline void beeperON() { digitalWrite(beeper_pin_, HIGH); }

  inline void beeperOFF() { digitalWrite(beeper_pin_, LOW); }
};


/// AlarmManager ///

class AlarmManager {
public:
  AlarmManager(const int& beeper_pin, const int& snooze_pin, Display* displ);

  void begin();

  void update();

  void high_pressure(bool value);

  void low_pressure(bool value);

  void bad_plateau(bool value);

private:
  const int beeper_pin_, snooze_pin_;
  Display* displ_;
};


} // namespace alarms


#endif

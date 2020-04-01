#ifndef Alarms_h
#define Alarms_h

#include "Arduino.h"

#include "Display.h"


namespace alarms {

// Time during which alarms are silenced, in milliseconds
static const unsigned long SNOOZE_TIME = 2 * 60 * 1000UL;

// Time each alarm is displayed if multiple, in milliseconds
static const unsigned long DISPLAY_TIME_EACH = 2 * 1000UL;


using display::Display;


/// DebouncedButton ///

class DebouncedButton {
  const unsigned long DEBOUNCE_DELAY = 50;

public:
  DebouncedButton(const int& pin): pin_(pin) {}

  int read() {
    int reading = digitalRead(pin_);

    if (reading != last_button_state_) {
      last_debounce_time_ = millis();
    }
    if ((millis() - last_debounce_time_) > DEBOUNCE_DELAY) {
      button_state_ = reading;
    }
    last_button_state_ = reading;
    return button_state_;
  }

private:
  int pin_;
  int last_button_state_ = LOW;
  int button_state_ = LOW;
  unsigned long last_debounce_time_ = 0;
};


/// Beeper ///

class Beeper {
public:
  Beeper(const int& beeper_pin, const int& snooze_pin);

  // Setup 
  void begin();

  // Update, should be called every loop
  void update();

  // Indicate that alarms are ON
  inline void alarmsON() { alarms_on_ = true; };

  // Indicate that alarms are OFF
  inline void alarmsOFF() { alarms_on_ = false; };

private:
  bool alarms_on_ = false;
  unsigned long snooze_time_ = 0;
  bool snoozed_ = false;

  const int beeper_pin_, snooze_pin_;
  DebouncedButton snooze_button_;

  bool snoozeButtonPressed() const;

  void toggleSnooze();

  inline void beeperON() { digitalWrite(beeper_pin_, HIGH); }

  inline void beeperOFF() { digitalWrite(beeper_pin_, LOW); }
};


/// Alarm ///

class Alarm {
public:
  Alarm(){};
  
  // Construct from pins and display
  Alarm(const String& text): text_(text) {}

  // Set ON value
  inline void setValue(const bool& on) { on_ = on; }

  // Check if this alarm is on
  inline bool is_ON() const { return on_; }

  // Get the text of this alarm
  inline const String text() const { return text_; }

private:
  String text_;
  bool on_ = false;
};


/// AlarmManager ///

class AlarmManager {
  enum Indices {
    HIGH_PRES_IDX = 0,
    LOW_PRES_IDX,
    BAD_PLAT_IDX,
    NUM_ALARMS 
  };
  const char* strings[NUM_ALARMS] = {
    "HIGH PRESURE",
    "LOW PRES DISCONNECT?",
    "ELEVATED PEAK PRES"
  };

public:
  AlarmManager(const int& beeper_pin, const int& snooze_pin, Display* displ);

  // Set up manager, should be called in setup()
  void begin();

  // Update alarms, should be called every loop
  void update();

  // Indicate state of high pressure alarm
  inline void high_pressure(const bool& value) { alarms_[HIGH_PRES_IDX].setValue(value); }

  // Indicate state of low pressure alarm
  inline void low_pressure(const bool& value) { alarms_[LOW_PRES_IDX].setValue(value); }

  // Indicate state of bad plateau alarm
  inline void bad_plateau(const bool& value) { alarms_[BAD_PLAT_IDX].setValue(value); }

private:
  Display* displ_;
  Beeper beeper_;
  Alarm alarms_[NUM_ALARMS];

  // Get number of alarms that are ON
  int numON() const;

  // Get text to display
  const String getText() const;
};


} // namespace alarms


#endif

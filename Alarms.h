#ifndef Alarms_h
#define Alarms_h

#include "Arduino.h"

#include "Display.h"
#include "pitches.h"

namespace alarms {

// Time during which alarms are silenced, in milliseconds
static const unsigned long SNOOZE_TIME = 2 * 60 * 1000UL;

// Time each alarm is displayed if multiple, in milliseconds
static const unsigned long DISPLAY_TIME_EACH = 2 * 1000UL;


using display::Display;


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
  unsigned long tone_timer_ = 0;
  int tone_step_ = notes_len_;

  // notes in the emergency alarm defines the pattern:
  static const int notes_len_ = 5;
  int notes_[notes_len_] = {NOTE_G4, NOTE_G4, NOTE_G4, NOTE_G4, NOTE_G5};
  int note_durations_[notes_len_] = {300, 300, 300, 200, 200};
  int note_pauses_[notes_len_] = {200, 200, 400, 100, 1500};

  const int beeper_pin_, snooze_pin_;

  bool snoozeButtonPressed() const;

  void toggleSnooze();

  inline void beeperPlay();
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

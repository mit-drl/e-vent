#ifndef Alarms_h
#define Alarms_h

#include "Arduino.h"

#include "Display.h"
#include "pitches.h"

namespace alarms {


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
 * Beeper
 * Represents the alarm speaker/buzzer. Handles playing of tones and snoozing.
 */
class Beeper {

  // Time during which alarms are silenced, in milliseconds
  static const unsigned long kSnoozeTime = 2 * 60 * 1000UL;

  // notes in the emergency alarm defines the pattern:
  static const int kNotesLen = 5;
  const int kNotes[kNotesLen] = {NOTE_G4, NOTE_G4, NOTE_G4, NOTE_G4, NOTE_G5};
  const int kNoteDurations[kNotesLen] = {300, 300, 300, 200, 200};
  const int kNotePauses[kNotesLen] = {200, 200, 400, 100, 1500};

public:
  Beeper(const int& beeper_pin, const int& snooze_pin);

  // Setup during arduino setup()
  void begin();

  // Update during arduino loop()
  void update();

  // Indicate that alarms are ON
  inline void alarmsON() { alarms_on_ = true; };

  // Indicate that alarms are OFF
  inline void alarmsOFF() { alarms_on_ = false; };

private:
  const int beeper_pin_;
  DebouncedButton snooze_button_;
  bool alarms_on_ = false;
  unsigned long snooze_time_ = 0;
  bool snoozed_ = false;
  unsigned long tone_timer_ = 0;
  int tone_step_ = kNotesLen;

  bool snoozeButtonPressed() const;

  void toggleSnooze();

  void play();
};


/** 
 * Alarm
 * Keeps track of the state of a specific alarm.
 */
class Alarm {
public:
  Alarm(){};
  
  Alarm(const String& text): text_(text) {}

  // Set the ON value of this alarm, but only turn off if `bad == false` for at least `min_ok` 
  // consecutive calls with different `seq`. The default is 1, meaning that a single call
  // in which `bad == false` clears the alarm.
  void setCondition(const bool& bad, const unsigned long& seq, const int& min_ok = 1) {
    if (bad) {
      on_ = true;
      consecutive_good_ = 0;
    } else {
      consecutive_good_ += (seq != last_good_seq_);
      last_good_seq_ = seq;
      if (on_) {
        on_ = consecutive_good_ < min_ok;
      }
    }
  }

  // Check if this alarm is on
  inline bool isON() const { return on_; }

  // Get the text of this alarm
  inline const String text() const { return text_; }

private:
  String text_;
  bool on_ = false;
  unsigned long consecutive_good_ = 0;
  unsigned long last_good_seq_ = 0;
};


using display::Display;

/**
 * AlarmManager
 * Manages multple alarms on the same screen space.
 * If there is one alarm on, its text blinks in a designated portion of the screen,
 * if there are more, each one blinks for `kDisplayTime` milliseconds at a time.
 */
class AlarmManager {

  // Time each alarm is displayed if multiple, in milliseconds
  static const unsigned long kDisplayTime = 2 * 1000UL;

  // Indices for the different alarms
  enum Indices {
    HIGH_PRES_IDX = 0,
    LOW_PRES_IDX,
    BAD_PLAT_IDX,
    UNMET_VOLUME,
    NO_TIDAL_PRES,
    OVER_CURRENT,
    NUM_ALARMS 
  };

  // Text to display for each alarm
  const char* strings[NUM_ALARMS] = {
    "    HIGH PRESURE    ",
    "LOW PRES DISCONNECT?",
    " ELEVATED PEAK PRES ",
    " UNMET TIDAL VOLUME ",
    " NO TIDAL PRESSURE  ",
    " OVER CURRENT FAULT "
  };

public:
  AlarmManager(const int& beeper_pin, const int& snooze_pin, 
               Display* displ, unsigned long const* cycle_count);

  // Setup during arduino setup()
  void begin();

  // Update alarms, should be called every loop
  void update();

  // Pressure too high alarm
  inline void highPressure(const bool& value) {
    alarms_[HIGH_PRES_IDX].setCondition(value, *cycle_count_, 2);
  }

  // Pressure too low alarm
  inline void lowPressure(const bool& value) {
    alarms_[LOW_PRES_IDX].setCondition(value, *cycle_count_, 1);
  }

  // Bad plateau alarm
  inline void badPlateau(const bool& value) {
    alarms_[BAD_PLAT_IDX].setCondition(value, *cycle_count_, 1);
  }

  // Tidal volume not met alarm
  inline void unmetVolume(const bool& value) {
    alarms_[UNMET_VOLUME].setCondition(value, *cycle_count_, 2);
  }

  // Tidal pressure not detected alarm
  // TODO only set after 2 bad cycles?
  inline void noTidalPres(const bool& value) {
    alarms_[NO_TIDAL_PRES].setCondition(value, *cycle_count_, 2);
  }

  // Current too high alarm
  inline void overCurrent(const bool& value) {
    alarms_[OVER_CURRENT].setCondition(value, *cycle_count_, 2);
  }

private:
  Display* displ_;
  Beeper beeper_;
  Alarm alarms_[NUM_ALARMS];
  unsigned long const* cycle_count_;

  // Get number of alarms that are ON
  int numON() const;

  // Get text to display
  const String getText() const;
};


} // namespace alarms


#endif

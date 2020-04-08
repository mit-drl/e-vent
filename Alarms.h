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
  
  Alarm(const String& text, const int& min_bad_to_trigger, const int& min_good_to_clear);

  // Set the ON value of this alarm, but only turn ON if `bad == true` for at least 
  // `min_bad_to_trigger_` consecutive calls with different `seq` and OFF if `bad == false` 
  // for at least `min_good_to_clear_` consecutive calls with different `seq`.   
  void setCondition(const bool& bad, const unsigned long& seq);

  // Check if this alarm is on
  inline bool isON() const { return on_; }

  // Get the text of this alarm
  inline const String text() const { return text_; }

private:
  String text_;
  bool on_ = false;
  int min_bad_to_trigger_;
  int min_good_to_clear_;
  unsigned long consecutive_bad_ = 0;
  unsigned long consecutive_good_ = 0;
  unsigned long last_bad_seq_ = 0;
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
    HIGH_PRESSU = 0,
    LOW_PRESSUR,
    BAD_PLATEAU,
    UNMET_VOLUM,
    NO_TIDAL_PR,
    OVER_CURREN,
    NUM_ALARMS 
  };

public:
  AlarmManager(const int& beeper_pin, const int& snooze_pin, 
               Display* displ, unsigned long const* cycle_count):
      displ_(displ),
      beeper_(beeper_pin, snooze_pin),
      cycle_count_(cycle_count) {
    alarms_[HIGH_PRESSU] = Alarm("    HIGH PRESURE    ", 1, 2);
    alarms_[LOW_PRESSUR] = Alarm("LOW PRES DISCONNECT?", 1, 1);
    alarms_[BAD_PLATEAU] = Alarm(" ELEVATED PEAK PRES ", 1, 1);
    alarms_[UNMET_VOLUM] = Alarm(" UNMET TIDAL VOLUME ", 1, 1);
    alarms_[NO_TIDAL_PR] = Alarm(" NO TIDAL PRESSURE  ", 2, 1);
    alarms_[OVER_CURREN] = Alarm(" OVER CURRENT FAULT ", 1, 2);
  }

  // Setup during arduino setup()
  void begin();

  // Update alarms, should be called every loop
  void update();

  // Pressure too high alarm
  inline void highPressure(const bool& value) {
    alarms_[HIGH_PRESSU].setCondition(value, *cycle_count_);
  }

  // Pressure too low alarm
  inline void lowPressure(const bool& value) {
    alarms_[LOW_PRESSUR].setCondition(value, *cycle_count_);
  }

  // Bad plateau alarm
  inline void badPlateau(const bool& value) {
    alarms_[BAD_PLATEAU].setCondition(value, *cycle_count_);
  }

  // Tidal volume not met alarm
  inline void unmetVolume(const bool& value) {
    alarms_[UNMET_VOLUM].setCondition(value, *cycle_count_);
  }

  // Tidal pressure not detected alarm
  inline void noTidalPres(const bool& value) {
    alarms_[NO_TIDAL_PR].setCondition(value, *cycle_count_);
  }

  // Current too high alarm
  inline void overCurrent(const bool& value) {
    alarms_[OVER_CURREN].setCondition(value, *cycle_count_);
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

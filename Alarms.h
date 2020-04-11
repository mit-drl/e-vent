#ifndef Alarms_h
#define Alarms_h

#include "Arduino.h"

#include "Display.h"
#include "pitches.h"


namespace alarms {


using display::Display;


// Alarm levels in order of increasing priority
enum AlarmLevel {
  NO_ALARM,
  NOTIFY,
  EMERGENCY,
  NUM_LEVELS
};


struct Note {
  int note;
  int duration;
  int pause;
};


static const Note kEmergencyNotes[] = {
  {NOTE_G4, 300, 200},
  {NOTE_G4, 300, 200},
  {NOTE_G4, 300, 400},
  {NOTE_G4, 200, 100},
  {NOTE_G5, 200, 1500}
};

static const Note kNotifyNotes[] = {
  {NOTE_G4, 300, 200},  // TODO design notes
  {NOTE_G4, 300, 200},
  {NOTE_G4, 300, 400}
};


class Tone {
public:
  Tone() {}

  Tone(const Note notes[], const int* pin): 
      notes_(notes),
      pin_(pin),
      length_(sizeof(notes) / sizeof(notes[0])),
      tone_step_(length_) {}

  void play();

  inline void stop() { tone_step_ = length_; }

private:
  Note* notes_;
  int* pin_;
  int length_;
  int tone_step_;
  unsigned long tone_timer_ = 0;
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


/**
 * Beeper
 * Represents the alarm speaker/buzzer. Handles playing of tones and snoozing.
 */
class Beeper {

  // Time during which alarms are silenced, in milliseconds
  static const unsigned long kSnoozeTime = 2 * 60 * 1000UL;

public:
  Beeper(const int& beeper_pin, const int& snooze_pin);

  // Setup during arduino setup()
  void begin();

  // Update during arduino loop()
  void update(const AlarmLevel& alarm_level);

private:
  const int beeper_pin_;
  DebouncedButton snooze_button_;
  Tone tones_[NUM_LEVELS];

  unsigned long snooze_time_ = 0;
  bool snoozed_ = false;

  bool snoozeButtonPressed() const;

  void toggleSnooze();

  void play(const AlarmLevel& alarm_level);

  void stop();
};


/** 
 * Alarm
 * Keeps track of the state of a specific alarm.
 */
class Alarm {
public:
  Alarm(){};
  
  Alarm(const String& text, const AlarmLevel& alarm_level,
        const int& min_bad_to_trigger, const int& min_good_to_clear);

  // Set the ON value of this alarm, but only turn ON if `bad == true` for at least 
  // `min_bad_to_trigger_` consecutive calls with different `seq` and OFF if `bad == false` 
  // for at least `min_good_to_clear_` consecutive calls with different `seq`.   
  void setCondition(const bool& bad, const unsigned long& seq);

  // Check if this alarm is on
  inline bool isON() const { return on_; }

  // Get the text of this alarm
  inline String text() const { return text_; }

  // Get the alarm level of this alarm
  inline AlarmLevel alarmLevel() const { return alarm_level_; }

private:
  String text_;
  AlarmLevel alarm_level_;
  int min_bad_to_trigger_;
  int min_good_to_clear_;
  bool on_ = false;
  unsigned long consecutive_bad_ = 0;
  unsigned long consecutive_good_ = 0;
  unsigned long last_bad_seq_ = 0;
  unsigned long last_good_seq_ = 0;
};


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
    alarms_[HIGH_PRESSU] = Alarm("    HIGH PRESURE    ", 1, 2, EMERGENCY);
    alarms_[LOW_PRESSUR] = Alarm("LOW PRES DISCONNECT?", 1, 1, EMERGENCY);
    alarms_[BAD_PLATEAU] = Alarm("  HIGH RESIST PRES  ", 1, 1, NOTIFY);
    alarms_[UNMET_VOLUM] = Alarm(" UNMET TIDAL VOLUME ", 1, 1, EMERGENCY);
    alarms_[NO_TIDAL_PR] = Alarm(" NO TIDAL PRESSURE  ", 2, 1, EMERGENCY);
    alarms_[OVER_CURREN] = Alarm(" OVER CURRENT FAULT ", 1, 2, EMERGENCY);
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
  String getText() const;

  // Get highest priority level of the alarms that are ON
  AlarmLevel getHighestLevel() const;
};


} // namespace alarms


#endif

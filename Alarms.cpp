#include "Alarms.h"


namespace alarms {


/// Beeper ///

Beeper::Beeper(const int& beeper_pin, const int& snooze_pin):
    beeper_pin_(beeper_pin), 
    snooze_pin_(snooze_pin),
    snooze_button_(snooze_pin) {}


void Beeper::begin() {
  pinMode(beeper_pin_, OUTPUT);
  pinMode(snooze_pin_, INPUT_PULLUP);
}

void Beeper::update() {
  if (snoozeButtonPressed()) {
    toggleSnooze();
  }
  // check if snooze time is up
  if (snoozed_ && millis() - snooze_time_ > SNOOZE_TIME) {
    snoozed_ = false;
  }
  if (alarms_on_ && !snoozed_) {
    if(tone_step_ == notes_len_) tone_step_ = 0; // Start again if tone finished
    beeperPlay();
  } else {
    tone_step_ = notes_len_;
  }
}

bool Beeper::snoozeButtonPressed() const {
  return snooze_button_.is_LOW();
}

void Beeper::toggleSnooze() {
  if (snoozed_) {
    snoozed_ = false;
  } else {
    snoozed_ = true;
    snooze_time_ = millis();
  }
}

void Beeper::beeperPlay(){
  unsigned long current = millis();
  if (tone_step_ == notes_len_) return; // The tone has completed
  if(current > tone_timer_){
    tone(beeper_pin_, notes_[tone_step_], note_durations_[tone_step_]);
    tone_timer_ = current + note_durations_[tone_step_] + note_pauses_[tone_step_];
    tone_step_ ++;
  }
}

/// AlarmManager ///

AlarmManager::AlarmManager(const int& beeper_pin, const int& snooze_pin, Display* displ):
    displ_(displ),
    beeper_(beeper_pin, snooze_pin) {
  for (int i = 0; i < NUM_ALARMS; i++) {
    alarms_[i] = Alarm(strings[i]);
  }
}

void AlarmManager::begin() {
  beeper_.begin();
}

void AlarmManager::update() {
  const String text = getText();
  displ_->setAlarm(text);
  if (text.length() > 0) {
    beeper_.alarmsON();
  } else {
    beeper_.alarmsOFF();
  }
  beeper_.update();
}

int AlarmManager::numON() const {
  int num = 0;
  for (int i = 0; i < NUM_ALARMS; i++) {
    num += (int)alarms_[i].is_ON();
  }
  return num;
}

const String AlarmManager::getText() const {
  int num_on = numON();
  String text = "";
  if (num_on > 0) {
    // determine which alarm to display
    int index = millis() % (numON() * DISPLAY_TIME_EACH) / DISPLAY_TIME_EACH;
    int count = 0;
    int i;
    for (i = 0; i < NUM_ALARMS; i++) {
      if (alarms_[i].is_ON()) {
        if (count++ == index) break;
      }
    }
    text = alarms_[i].text();
  }
  return text;
}


} // namespace alarms

#include "Alarms.h"


namespace alarms {


/// DebouncedButton ///

DebouncedButton::DebouncedButton(const int& pin): pin_(pin) {}

void DebouncedButton::begin() {
  pinMode(pin_, INPUT_PULLUP);
}

bool DebouncedButton::is_LOW() {
  int reading = digitalRead(pin_);

  bool low_value = false;
  const unsigned long time_now = millis();
  if (reading == LOW) {
    if ((time_now - last_low_time_) > kDebounceDelay) {
      low_value = true;
    }
    last_low_time_ = time_now;
  }
  return low_value;
}


/// Beeper ///

Beeper::Beeper(const int& beeper_pin, const int& snooze_pin):
    beeper_pin_(beeper_pin), 
    snooze_button_(snooze_pin) {}

void Beeper::begin() {
  snooze_button_.begin();
  pinMode(beeper_pin_, OUTPUT);
}

void Beeper::update() {
  if (snoozeButtonPressed()) {
    toggleSnooze();
  }
  // check if snooze time is up
  if (snoozed_ && millis() - snooze_time_ > kSnoozeTime) {
    snoozed_ = false;
  }
  if (alarms_on_ && !snoozed_) {
    if(tone_step_ == kNotesLen) tone_step_ = 0; // Start again if tone finished
    play();
  } else {
    tone_step_ = kNotesLen;
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

void Beeper::play(){
  unsigned long current = millis();
  if (tone_step_ == kNotesLen) return; // The tone has completed
  if(current > tone_timer_){
    tone(beeper_pin_, kNotes[tone_step_], kNoteDurations[tone_step_]);
    tone_timer_ = current + kNoteDurations[tone_step_] + kNotePauses[tone_step_];
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
  const int num_on = numON();
  String text = "";
  if (num_on > 0) {
    // determine which of the on alarms to display
    const int index = millis() % (num_on * kDisplayTime) / kDisplayTime;
    int count_on = 0;
    int i;
    for (i = 0; i < NUM_ALARMS; i++) {
      if (alarms_[i].is_ON()) {
        if (count_on++ == index) break;
      }
    }
    text = alarms_[i].text();
  }
  return text;
}


} // namespace alarms

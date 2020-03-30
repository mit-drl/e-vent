#include "Alarms.h"


namespace alarms {


Alarm::Alarm(const int& beeper_pin, Display* displ): 
    beeper_pin_(beeper_pin), 
    displ_(displ) {
  pinMode(beeper_pin, OUTPUT);
}

void Alarm::update() {
  // check if snooze time is up
  if (snoozed_ && millis() - snooze_time_ > SNOOZE_TIME) {
    snoozed_ = false;
  }
  if (beep_ && !snoozed_) {
    beeperON();
  } else {
    beeperOFF();
  }
  displ_->setAlarm(text_);
}

void Alarm::loud(const String& alarm_message) {
  // need to clear snooze if another alarm was already snoozed
  if (alarm_message != text_){
    snoozed_ = false;
  }
  beep_ = true;
  text_ = alarm_message;
}

void Alarm::silent(const String& alarm_message) {
  beep_ = false;
  text_ = alarm_message;
}

void Alarm::clear() {
  beep_ = false;
  text_ = "";
}

void Alarm::snooze() {
  if (!snoozed_) {
    snoozed_ = true;
    snooze_time_ = millis();
  }
}


} // namespace alarms


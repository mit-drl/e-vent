#include "Alarms.h"


namespace alarms {


Alarm::Alarm(const int& beeper_pin, Display* displ): 
    beeper_pin_(beeper_pin), 
    displ_(displ),
    beep_(false) {
  pinMode(beeper_pin, OUTPUT);
}

void Alarm::update() {
  if (beep_) {
    beeperON();
  } else {
    beeperOFF();
  }
  displ_->setAlarm(text_);
}

void Alarm::loud(const String& alarm_message) {
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
  // TODO
}


} // namespace alarms


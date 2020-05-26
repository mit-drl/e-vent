/**
 * MIT Emergency Ventilator Controller
 * 
 * MIT License:
 * 
 * Copyright (c) 2020 MIT
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * Alarms.cpp
 */

#include "Alarms.h"


namespace alarms {


/// Tone ///

Tone::Tone(const Note notes[], const int& notes_length, const int* pin): 
    notes_(notes),
    pin_(pin),
    length_(notes_length),
    tone_step_(length_) {}

void Tone::play() {
  if (length_ == 0) {
    return;
  }
  if (!playing_) {  // Do once when tone starts
    tone_timer_ = millis();
    tone_step_ = 0;
    playing_ = true;
  }
  tone_step_ %= length_; // Start again if tone finished
  if (millis() > tone_timer_) {
    tone(*pin_, notes_[tone_step_].note, notes_[tone_step_].duration);
    tone_timer_ += notes_[tone_step_].duration + notes_[tone_step_].pause;
    tone_step_ ++;
  }
}


/// Beeper ///

void Beeper::begin() {
  snooze_button_.begin();
  pinMode(beeper_pin_, OUTPUT);
}

void Beeper::update(const AlarmLevel& alarm_level) {
  if (snoozeButtonPressed()) {
    toggleSnooze();
  }
  // check if snooze time is up
  if (snoozed_ && millis() - snooze_time_ > kSnoozeTime) {
    snoozed_ = false;
  }
  if (snoozed_) {
    stop();
  } else {
    play(alarm_level);
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

void Beeper::play(const AlarmLevel& alarm_level) {
  for (int i = 0; i < NUM_LEVELS; i++) {
    if (i != alarm_level) {
      tones_[i].stop();
    }
  }
  tones_[alarm_level].play();
}

void Beeper::stop() {
  for (int i = 0; i < NUM_LEVELS; i++) {
    tones_[i].stop();
  }
}


/// Alarm ///

Alarm::Alarm(const String& default_text, const int& min_bad_to_trigger,
             const int& min_good_to_clear, const AlarmLevel& alarm_level):
  text_(default_text),
  min_bad_to_trigger_(min_bad_to_trigger),
  min_good_to_clear_(min_good_to_clear),
  alarm_level_(alarm_level) {}

void Alarm::reset() {
  *this = Alarm::Alarm(text_, min_bad_to_trigger_, min_good_to_clear_, alarm_level_);
}

void Alarm::setCondition(const bool& bad, const unsigned long& seq) {
  if (bad) {
    consecutive_bad_ += (seq != last_bad_seq_);
    last_bad_seq_ = seq;
    if (!on_) {
      on_ = consecutive_bad_ >= min_bad_to_trigger_;
    }
    consecutive_good_ = 0;
  } else {
    consecutive_good_ += (seq != last_good_seq_);
    last_good_seq_ = seq;
    if (on_) {
      on_ = consecutive_good_ < min_good_to_clear_;
    }
    consecutive_bad_ = 0;
  }
}

void Alarm::setText(const String& text) {
  if (text.length() == display::kWidth) {
    text_ = text;
  }
  else if (text.length() > display::kWidth) {
    text_ = text.substring(0, display::kWidth);
  }
  else {
    text_ = text;
    while (text_.length() < display::kWidth) {
      text_ += " ";
    }
  }
}


/// AlarmManager ///

void AlarmManager::begin() {
  beeper_.begin();
  pinMode(led_pin_, OUTPUT);
}

void AlarmManager::update() {
  displ_->setAlarmText(getText());
  AlarmLevel highest_level = getHighestLevel();
  beeper_.update(highest_level);
  if (highest_level > NO_ALARM) {
    digitalWrite(led_pin_, led_pulse_.read() ? HIGH : LOW);
  }
  else {
    digitalWrite(led_pin_, LOW);
  }
}

void AlarmManager::allOff() {
  for (int i = 0; i < NUM_ALARMS; i++) {
    alarms_[i].reset();
  }
}

int AlarmManager::numON() const {
  int num = 0;
  for (int i = 0; i < NUM_ALARMS; i++) {
    num += (int)alarms_[i].isON();
  }
  return num;
}

String AlarmManager::getText() const {
  const int num_on = numON();
  String text = "";
  if (num_on > 0) {
    // determine which of the on alarms to display
    const int index = millis() % (num_on * kDisplayTime) / kDisplayTime;
    int count_on = 0;
    int i;
    for (i = 0; i < NUM_ALARMS; i++) {
      if (alarms_[i].isON()) {
        if (count_on++ == index) break;
      }
    }
    text = alarms_[i].text();
  }
  return text;
}

AlarmLevel AlarmManager::getHighestLevel() const {
  AlarmLevel alarm_level = NO_ALARM;
  for (int i = 0; i < NUM_ALARMS; i++) {
    if (alarms_[i].isON()) {
      alarm_level = max(alarm_level, alarms_[i].alarmLevel());
    }
  }
  return alarm_level;
}


} // namespace alarms

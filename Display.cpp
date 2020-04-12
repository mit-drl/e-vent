#include "Display.h"

namespace display {


/// TextAntimation ///

void TextAnimation::reset(const String& text) {
  reset_time_ = millis();
  text_ = text;
}

bool TextAnimation::empty() {
  return text_.length() == 0;
}

const String& TextAnimation::text() {
  return text_;
}

const String TextAnimation::getLine() {
  String new_text;
  unsigned long time_now = millis();
  if(time_now - reset_time_ < kBlinkOnFraction * kBlinkPeriod) {
    new_text = text_;
    while(new_text.length() < kWidth) {
      new_text += " ";
    }
    if(new_text.length() > kWidth) {
      new_text = new_text.substring(0, 20);
    }
  }
  else if(time_now - reset_time_ < kBlinkPeriod) {
    new_text = kBlankLine;
  }
  else {
    reset_time_ = time_now;
  }
  return new_text;
}


/// Display ///

void Display::begin() {
  lcd_->begin(kWidth, kHeight);
  lcd_->noCursor(); 
  update();
}

void Display::update() {
  writeHeader();
}

void Display::setAlarmText(const String& alarm) {
  if(animation_.text() != alarm) {
    animation_.reset(alarm);
  }
}

template <typename T>
void Display::write(const DisplayKey& key, const T& value) {
  switch (key) {
    case HEADER:
      writeHeader();
      break;
    case VOLUME:
      writeVolume(value);
      break;
    case BPM:
      writeBPM(value);
      break;
    case IE_RATIO:
      writeIEratio(value);
      break;
    case AC_TRIGGER:
      writeACTrigger(value);
      break;
    case PRES_LABEL:
      writePresLabel();
      break;
    case PEAK_PRES:
      writePeakP(value);
      break;
    case PLATEAU_PRES:
      writePlateauP(value);
      break;
    case PEEP_PRES:
      writePEEP(value);
      break;
  }
}

void Display::writeBlank(const DisplayKey& key) {
  write(elements_[key].row, elements_[key].col, elements_[key].blank);
}

void Display::writeHeader() {
  if(animation_.empty()) {
    writePresLabel();
  } 
  else {
    write(elements_[HEADER].row, elements_[HEADER].col, animation_.getLine());
  }
}

void Display::writeVolume(const int& vol) {
  if(animation_.empty()) {
    char buff[12];
    sprintf(buff, "V=%3d mL   ", vol);
    write(elements_[VOLUME].row, elements_[VOLUME].col, buff);
  }
}

void Display::writeBPM(const int& bpm) {
  char buff[12];
  sprintf(buff, "RR=%2d/min  ", bpm);
  write(elements_[BPM].row, elements_[BPM].col, buff);
}

void Display::writeIEratio(const float& ie) {
  char ie_buff[4];
  dtostrf(ie, 3, 1, ie_buff);
  char buff[12];
  sprintf(buff, "I:E=1:%s  ", ie_buff);
  write(elements_[IE_RATIO].row, elements_[IE_RATIO].col, buff);
}

void Display::writeACTrigger(const float& ac_trigger) {
  char buff[12];
  if(ac_trigger > trigger_threshold_) {
    char ac_buff[4];
    dtostrf(ac_trigger, 3, 1, ac_buff);
    sprintf(buff, "AC=%scmH20", ac_buff);
  } else {
    sprintf(buff, "AC=OFF    ");
  }
  write(elements_[AC_TRIGGER].row, elements_[AC_TRIGGER].col, buff);
}

void Display::writePresLabel() {
  write(elements_[PRES_LABEL].row, elements_[PRES_LABEL].col, " P(cmH2O):");
}

void Display::writePeakP(const int& peak) {
  char buff[10];
  sprintf(buff, "  peak=%2d", peak);
  write(elements_[PEAK_PRES].row, elements_[PEAK_PRES].col, buff);
}

void Display::writePlateauP(const int& plat) {
  char buff[10];
  sprintf(buff, "  plat=%2d", plat);
  write(elements_[PLATEAU_PRES].row, elements_[PLATEAU_PRES].col, buff);
}

void Display::writePEEP(const int& peep) {
  char buff[10];
  sprintf(buff, "  PEEP=%2d", peep);
  write(elements_[PEEP_PRES].row, elements_[PEEP_PRES].col, buff);
}

template <typename T>
void Display::write(const int& row, const int& col, const T& printable) {
  lcd_->setCursor(col, row);
  lcd_->print(printable);
}


}  // namespace display

#include "Display.h"

namespace display {


/// TextAntimation ///

void TextAnimation::reset(const String& text){
  reset_time_ = millis();
  text_ = text;
}

bool TextAnimation::empty(){
  return text_.length() == 0;
}

const String& TextAnimation::text(){
  return text_;
}

const String TextAnimation::getLine(){
  String new_text;
  unsigned long time_now = millis();
  if(time_now - reset_time_ < kBlinkOnFraction * kBlinkPeriod){
    new_text = text_;
    while(new_text.length() < kWidth){
      new_text += " ";
    }
    if(new_text.length() > kWidth){
      new_text = new_text.substring(0, 20);
    }
  }
  else if(time_now - reset_time_ < kBlinkPeriod){
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

void Display::update(){
  if(animation_.empty()){
    write(PRES_LABEL, " P(cmH2O):");
  } 
  else {
    write(HEADER, animation_.getLine());
  }
}

void Display::write(const DisplayKey& key, const String& printable){
  String s = printable;
  if (printable.length() > elements_[key].width) {
    s = s.substring(0, elements_[key].width);
  }
  while (s.length() < elements_[key].width) {
    s += " ";
  }
  write(elements_[key].row, elements_[key].col, s);
}

void Display::writeAlarmText(const String& alarm){
  if(animation_.text() != alarm){
    animation_.reset(alarm);
  }
}

void Display::writeVolume(const int& vol){
  if(animation_.empty()){
    char buff[12];
    sprintf(buff, "V=%3d mL   ", vol);
    write(VOLUME, buff);
  }
}

void Display::writeBPM(const int& bpm){
  char buff[12];
  sprintf(buff, "RR=%2d/min  ", bpm);
  write(BPM, buff);
}

void Display::writeIEratio(const float& ie){
  char ie_buff[4];
  dtostrf(ie, 3, 1, ie_buff);
  char buff[12];
  sprintf(buff, "I:E=1:%s  ", ie_buff);
  write(IE_RATIO, buff);
}

void Display::writeACTrigger(const float& ac_trigger, const float& lower_threshold){
  char buff[12];
  if(ac_trigger > lower_threshold){
    char ac_buff[4];
    dtostrf(ac_trigger, 3, 1, ac_buff);
    sprintf(buff, "AC=%scmH20", ac_buff);
  } else {
    sprintf(buff, "AC=OFF    ");
  }
  write(AC_TRIGGER, buff);
}

void Display::writePeakP(const int& peak){
  char buff[10];
  sprintf(buff, "  peak=%2d", peak);
  write(PEAK_PRES, buff);
}

void Display::writePlateauP(const int& plat){
  char buff[10];
  sprintf(buff, "  plat=%2d", plat);
  write(PLATEAU_PRES, buff);
}

void Display::writePEEP(const int& peep){
  char buff[10];
  sprintf(buff, "  PEEP=%2d", peep);
  write(PEEP_PRES, buff);
}

template <typename T>
void Display::write(const int& row, const int& col, const T& printable){
  lcd_->setCursor(col, row);
  lcd_->print(printable);
}


}  // namespace display

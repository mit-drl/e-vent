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
  if(time_now - reset_time_ < BLINK_ON_FRACTION * BLINK_PERIOD){
    new_text = text_;
    while(new_text.length() < WIDTH){
      new_text += " ";
    }
    if(new_text.length() > WIDTH){
      new_text = new_text.substring(0, 20);
    }
  }
  else if(time_now - reset_time_ < BLINK_PERIOD){
    new_text = BLANK_LINE;
  }
  else {
    reset_time_ = time_now;
  }
  return new_text;
}


/// Display ///

Display::Display(LiquidCrystal* lcd): lcd_(lcd) {}

void Display::begin() {
  lcd_->begin(WIDTH, HEIGHT);
  lcd_->noCursor(); 
  update();
}

void Display::update(){
  if(animation_.empty()){
    write(0, 0, DEFAULT_HEADER);
  } 
  else {
    write(0, 0, animation_.getLine());
  }
}

void Display::setAlarm(const String& alarm){
  if(animation_.text() != alarm){
    animation_.reset(alarm);
  }
}

void Display::clearAlarm(){
  animation_.reset();
}

void Display::writeVolume(const int& vol){
  char buff[12];
  sprintf(buff, " V=%2d%% max ", vol);
  write(1, 0, buff);
}

void Display::writeBPM(const int& bpm){
  char buff[12];
  sprintf(buff, " RR=%2d/min ", bpm);
  write(2, 0, buff);
}

void Display::writeIEratio(const float& ie){
  char buff[4];
  sprintf(buff, " I:E=1:%3.1f ", (double)ie);
  write(3, 0, buff);
}

void Display::writePeakP(const int& peak){
  char buff[10];
  sprintf(buff, "  peak=%2d", peak);
  write(1, 11, buff);
}

void Display::writePlateauP(const int& plat){
  char buff[10];
  sprintf(buff, "  plat=%2d", plat);
  write(2, 11, buff);
}

void Display::writePEEP(const int& peep){
  char buff[10];
  sprintf(buff, "  PEEP=%2d", peep);
  write(3, 11, buff);
}

template <typename T>
void Display::write(const int& row, const int& col, const T& printable){
  lcd_->setCursor(col, row);
  lcd_->print(printable);
}


}  // namespace display

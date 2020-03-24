#include "Display.h"

Display::Display(LiquidCrystal* lcd): lcd_(lcd) {}

void Display::begin() {
  lcd_->begin(20, 4);
  lcd_->noCursor(); 

  write(0, 0, "Set:       P(cmH2O):", 20);
  write(1, 0, " V=  % max   peak=  ", 20);
  write(2, 0, " RR=  /min   plat=  ", 20);
  write(3, 0, " I:E=1:      PEEP=  ", 20);
}

void Display::writeVolume(const int& vol){
  char buff[3];
  sprintf(buff, "%2d", vol);
  write(1, 3, buff, 2);  // write volume at row 1, col 3
}

void Display::writeBPM(const int& bpm){
  char buff[3];
  sprintf(buff, "%2d", bpm);
  write(2, 4, buff, 2);  // write BPM at row 2, col 4
}

void Display::writeIEratio(const float& ie){
  char buff[4];
  dtostrf(ie, 3, 1, buff);
  write(3, 7, buff, 3);  // write IE at row 3, col 7
}

void Display::writePeakP(const int& peak){
  char buff[3];
  sprintf(buff, "%2d", peak);
  write(1, 18, buff, 2);  // write peak P at row 1, col 18
}

void Display::writePlateauP(const int& plat){
  char buff[3];
  sprintf(buff, "%2d", plat);
  write(2, 18, buff, 2);  // write plateau P at row 2, col 18
}

void Display::writePEEP(const int& peep){
  char buff[3];
  sprintf(buff, "%2d", peep);
  write(3, 18, buff, 2);  // write PEEP P at row 3, col 18
}

template <typename T>
void Display::write(const int& row, const int& col, const T& printable, const int& width){
  for(int i=0; i<width; i++){
    lcd_->setCursor(col + i, row);
    lcd_->write(' ');
  }
  lcd_->setCursor(col, row);
  lcd_->print(printable);
}

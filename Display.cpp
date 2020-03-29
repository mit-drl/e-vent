#include "Display.h"

Display::Display(LiquidCrystal* lcd): lcd_(lcd) {}

void Display::begin() {
  lcd_->begin(20, 4);
  lcd_->noCursor(); 
  writeHeader();
}

void Display::writeHeader(){
  write(0, 0, "Set:       P(cmH2O):");
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

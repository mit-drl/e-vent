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
 * Display.cpp
 */

#include "Display.h"

namespace display {


/// Display ///

void Display::begin() {
  lcd_->begin(kWidth, kHeight);
  lcd_->noCursor(); 
  addCustomCharacters();
  update();
}

void Display::addCustomCharacters() {
  lcd_->createChar(0, patientIcon);
  lcd_->createChar(1, timeIcon);
  lcd_->createChar(2, peakIcon);
  lcd_->createChar(3, plateauIcon);
  lcd_->createChar(4, peepIcon);
  lcd_->createChar(5, ieiIcon);
  lcd_->createChar(6, ie1Icon);
  lcd_->createChar(7, bellIcon);
}

void Display::update() {
  writeHeader();
  writeFooter();
}

void Display::setAlarmText(const String& alarm) {
  if (animation_.text() != alarm) {
    animation_.reset(alarm);
  }
}

void Display::setUnconfirmedKnobAlarmText(const String& alarm) {
  if (animationfooter_.text() != alarm) {
    animationfooter_.reset(alarm);
  }
}

void Display::setSnoozeText(const int& countdown) {
  snoozecountdown_ = countdown;  
}

template <typename T>
void Display::write(const DisplayKey& key, const T& value) {
  // Do not write on top of alarms
  if (alarmsON() && elements_[key].row == 0 && key != HEADER) {
    return;
  }
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
  // Do not write on top of alarms
  if (alarmsON() && elements_[key].row == 0 && key != HEADER) {
    return;
  }
  write(elements_[key].row, elements_[key].col, elements_[key].blank);
}

void Display::writeHeader() {
  if (!alarmsON()) {
    write(elements_[HEADER].row, elements_[HEADER].col, "Running...         ");
  } 
  else {
    const String line = animation_.getLine();
    if (line.length() > 0) {
      write(elements_[HEADER].row, elements_[HEADER].col, line);
    }
    else {
      writeBlank(HEADER);
    }
  }
}

void Display::writeFooter() {
  if (!alarmsON()) {
    writeBlank(FOOTER);
    hideBellIcon();
  } 
  else {
    showBellIcon();
    const String line = animationfooter_.getLine();
    if (line.length() > 0) {
      write(elements_[FOOTER].row, elements_[FOOTER].col, line);
    }
    else {
      writeBlank(FOOTER);
    }
  }
  writeSnoozeTime();
}

void Display::writeSnoozeTime() {
  char buff[7];
  if (snoozecountdown_>0){
    //sprintf(buff, "%3s", toString(SNOOZE, snoozecountdown_).c_str());
    sprintf(buff, "%3lu%   ", snoozecountdown_);
  } else {
    sprintf(buff, "%6s", "      ");
  }
  write(elements_[SNOOZE].row, elements_[SNOOZE].col, buff);
}

void Display::writeVolume(const int& vol) {
  const int vol_c = constrain(vol, 0, 999);
  char buff[5];
  sprintf(buff, "%2s  ", getLabel(VOLUME).c_str());
  write(elements_[VOLUME].row, elements_[VOLUME].col, buff);
  sprintf(buff, "%3s ", toString(VOLUME, vol_c).c_str());
  write(elements_[VOLUME].row+1, elements_[VOLUME].col, buff);
}

void Display::writeBPM(const int& bpm) {
  const int bpm_c = constrain(bpm, 0, 99);
  char buff[4];
  sprintf(buff, "%2s ", getLabel(BPM).c_str());
  write(elements_[BPM].row, elements_[BPM].col, buff);
  sprintf(buff, "%2s ", toString(BPM, bpm_c).c_str());
  write(elements_[BPM].row+1, elements_[BPM].col, buff);
}

void Display::writeIEratio(const float& ie) {
  const float ie_c = constrain(ie, 0.0, 9.9);
  showIEiIcon(elements_[IE_RATIO].row, elements_[IE_RATIO].col);
  showIE1Icon(elements_[IE_RATIO].row+1, elements_[IE_RATIO].col);
  char buff[5];
  sprintf(buff, "E   ");
  write(elements_[IE_RATIO].row, elements_[IE_RATIO].col+1, buff);
  sprintf(buff, "%3s ", toString(IE_RATIO, ie_c).c_str());
  write(elements_[IE_RATIO].row+1, elements_[IE_RATIO].col+1, buff);
}

void Display::writeACTrigger(const float& ac_trigger) {
  const float ac_trigger_c = constrain(ac_trigger, 0.0, 9.9);
  char buff[6];
  const String trigger_str = toString(AC_TRIGGER, ac_trigger_c);
  sprintf(buff, "%3s  ", getLabel(AC_TRIGGER).c_str());
  write(elements_[AC_TRIGGER].row, elements_[AC_TRIGGER].col, buff);
  sprintf(buff, "%3s  ", trigger_str.c_str());
  write(elements_[AC_TRIGGER].row+1, elements_[AC_TRIGGER].col, buff);
}

void Display::writePeakP(const int& peak) {
  const int peak_c = constrain(peak, -9, 99);
  showPeakIcon(elements_[PEAK_PRES].row, elements_[PEAK_PRES].col);
  char buff[3];
  sprintf(buff, "%2s", toString(PEAK_PRES, peak_c).c_str());
  write(elements_[PEAK_PRES].row, elements_[PEAK_PRES].col+1, buff);
}

void Display::writePlateauP(const int& plat) {
  const int plat_c = constrain(plat, -9, 99);
  showPlateauIcon(elements_[PLATEAU_PRES].row, elements_[PLATEAU_PRES].col);
  char buff[3];
  sprintf(buff, "%2s", toString(PLATEAU_PRES, plat_c).c_str());
  write(elements_[PLATEAU_PRES].row, elements_[PLATEAU_PRES].col+1, buff);
}

void Display::writePEEP(const int& peep) {
  const int peep_c = constrain(peep, -9, 99);
  showPEEPIcon(elements_[PEEP_PRES].row, elements_[PEEP_PRES].col);
  char buff[3];
  sprintf(buff, "%2s", toString(PEEP_PRES, peep_c).c_str());
  write(elements_[PEEP_PRES].row, elements_[PEEP_PRES].col+1, buff);
}

template <typename T>
String Display::toString(const DisplayKey& key, const T& value) const {
  switch (key) {
    case VOLUME:
      return String(value);
    case BPM:
      return String(value);
    case IE_RATIO:
      return String(value, 1);
    case AC_TRIGGER:
      return (value > trigger_threshold_ - 1e-2) ? String(value, 1) : "OFF";
    case PEAK_PRES:
      return String(value);
    case PLATEAU_PRES:
      return String(value);
    case PEEP_PRES:
      return String(value);
    case SNOOZE:
      return String(value);
    default:
      // Not meant to be used for other keys
      return "N/A";
  }
}

// This displays a patient icon
void Display::showPatientIcon() {
  lcd_->setCursor(elements_[BREATHING].col, elements_[BREATHING].row);
  lcd_->write(byte(0));
}

// This displays a time icon
void Display::showTimeIcon() {
  lcd_->setCursor(elements_[BREATHING].col, elements_[BREATHING].row);
  lcd_->write(byte(1));
}

// This hides a time-patient icon
void Display::hideTimePatientIcon() {
  hideIcon(elements_[BREATHING].row, elements_[BREATHING].col);
}

// Hides icon
void Display::hideIcon(const int& row, const int& col) {
  lcd_->setCursor(col, row);
  lcd_->write(byte(32));
}

// This displays the PIP icon
void Display::showPeakIcon(const int& row, const int& col) {
  lcd_->setCursor(col, row);
  lcd_->write(byte(2));
}

// This displays a plateau pressure icon
void Display::showPlateauIcon(const int& row, const int& col) {
  lcd_->setCursor(col, row);
  lcd_->write(byte(3));
}

// This displays a PEEP icon
void Display::showPEEPIcon(const int& row, const int& col) {
  lcd_->setCursor(col, row);
  lcd_->write(byte(4));
}

// This displays a I: icon
void Display::showIEiIcon(const int& row, const int& col) {
  lcd_->setCursor(col, row);
  lcd_->write(byte(5));
}

// This displays 1: icon
void Display::showIE1Icon(const int& row, const int& col) {
  lcd_->setCursor(col, row);
  lcd_->write(byte(6));
}

// This displays a bell icon
void Display::showBellIcon() {
  lcd_->setCursor(elements_[BELL].col, elements_[BELL].row);
  lcd_->write(byte(7));
}

// This hides the bell icon
void Display::hideBellIcon() {
  hideIcon(elements_[BELL].row, elements_[BELL].col);
}

template <typename T>
void Display::write(const int& row, const int& col, const T& printable) {
  lcd_->setCursor(col, row);
  lcd_->print(printable);
}


}  // namespace display

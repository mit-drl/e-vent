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
  update();
}

void Display::update() {
  writeHeader();
}

void Display::setAlarmText(const String& alarm) {
  if (animation_.text() != alarm) {
    animation_.reset(alarm);
  }
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
  // Do not write on top of alarms
  if (alarmsON() && elements_[key].row == 0 && key != HEADER) {
    return;
  }
  write(elements_[key].row, elements_[key].col, elements_[key].blank);
}

void Display::writeHeader() {
  if (!alarmsON()) {
    writePresLabel();
  } 
  else {
    const String line = animation_.getLine();
    if (line.length() > 0) {
      write(elements_[HEADER].row, elements_[HEADER].col, animation_.getLine());
    }
    else {
      writeBlank(HEADER);
    }
  }
}

void Display::writeVolume(const int& vol) {
  const int vol_c = constrain(vol, 0, 999);
  char buff[12];
  sprintf(buff, "%2s=%3s     ", getLabel(VOLUME).c_str(), toString(VOLUME, vol_c).c_str());
  write(elements_[VOLUME].row, elements_[VOLUME].col, buff);
}

void Display::writeBPM(const int& bpm) {
  const int bpm_c = constrain(bpm, 0, 99);
  char buff[12];
  sprintf(buff, "%2s=%2s      ", getLabel(BPM).c_str(), toString(BPM, bpm_c).c_str());
  write(elements_[BPM].row, elements_[BPM].col, buff);
}

void Display::writeIEratio(const float& ie) {
  const float ie_c = constrain(ie, 0.0, 9.9);
  char buff[12];
  sprintf(buff, "%2s=1:%3s   ", getLabel(IE_RATIO).c_str(), toString(IE_RATIO, ie_c).c_str());
  write(elements_[IE_RATIO].row, elements_[IE_RATIO].col, buff);
}

void Display::writeACTrigger(const float& ac_trigger) {
  if (animation_.empty()) {
    const float ac_trigger_c = constrain(ac_trigger, 0.0, 9.9);
    char buff[12];
    const String trigger_str = toString(AC_TRIGGER, ac_trigger_c);
    sprintf(buff, "%2s=%3s     ", getLabel(AC_TRIGGER).c_str(), trigger_str.c_str());
    write(elements_[AC_TRIGGER].row, elements_[AC_TRIGGER].col, buff);
  }

}

void Display::writePresLabel() {
  write(elements_[PRES_LABEL].row, elements_[PRES_LABEL].col, "Pressure:");
}

void Display::writePeakP(const int& peak) {
  const int peak_c = constrain(peak, -9, 99);
  char buff[10];
  sprintf(buff, "  %4s=%2s", getLabel(PEAK_PRES).c_str(), toString(PEAK_PRES, peak_c).c_str());
  write(elements_[PEAK_PRES].row, elements_[PEAK_PRES].col, buff);
}

void Display::writePlateauP(const int& plat) {
  const int plat_c = constrain(plat, -9, 99);
  char buff[10];
  sprintf(buff, "  %4s=%2s", getLabel(PLATEAU_PRES).c_str(),
          toString(PLATEAU_PRES, plat_c).c_str());
  write(elements_[PLATEAU_PRES].row, elements_[PLATEAU_PRES].col, buff);
}

void Display::writePEEP(const int& peep) {
  const int peep_c = constrain(peep, -9, 99);
  char buff[10];
  sprintf(buff, "  %4s=%2s", getLabel(PEEP_PRES).c_str(), toString(PEEP_PRES, peep_c).c_str());
  write(elements_[PEEP_PRES].row, elements_[PEEP_PRES].col, buff);
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
    default:
      // Not meant to be used for other keys
      return "N/A";
  }
}

template <typename T>
void Display::write(const int& row, const int& col, const T& printable) {
  lcd_->setCursor(col, row);
  lcd_->print(printable);
}


}  // namespace display

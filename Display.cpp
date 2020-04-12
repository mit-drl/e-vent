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
  if(animation_.empty()) {
    char buff[12];
    sprintf(buff, "%1s=%3s mL   ", getLabel(VOLUME).c_str(), toString(VOLUME, vol).c_str());
    write(elements_[VOLUME].row, elements_[VOLUME].col, buff);
  }
}

void Display::writeBPM(const int& bpm) {
  char buff[12];
  sprintf(buff, "%2s=%2s/min  ", getLabel(BPM).c_str(), toString(VOLUME, bpm).c_str());
  write(elements_[BPM].row, elements_[BPM].col, buff);
}

void Display::writeIEratio(const float& ie) {
  char buff[12];
  sprintf(buff, "%3s=1:%3s  ", getLabel(IE_RATIO).c_str(), toString(IE_RATIO, ie).c_str());
  write(elements_[IE_RATIO].row, elements_[IE_RATIO].col, buff);
}

void Display::writeACTrigger(const float& ac_trigger) {
  char buff[12];
  const String trigger_str = toString(AC_TRIGGER, ac_trigger);
  sprintf(buff, "%2s=%3s%5s", getLabel(AC_TRIGGER).c_str(), trigger_str.c_str(),
          trigger_str == "OFF" ? "     " : "cmH2O");
  write(elements_[AC_TRIGGER].row, elements_[AC_TRIGGER].col, buff);
}

void Display::writePresLabel() {
  write(elements_[PRES_LABEL].row, elements_[PRES_LABEL].col, " P(cmH2O):");
}

void Display::writePeakP(const int& peak) {
  char buff[10];
  sprintf(buff, "  %4s=%2s", getLabel(PEAK_PRES).c_str(), toString(PEAK_PRES, peak).c_str());
  write(elements_[PEAK_PRES].row, elements_[PEAK_PRES].col, buff);
}

void Display::writePlateauP(const int& plat) {
  char buff[10];
  sprintf(buff, "  %4s=%2s", getLabel(PLATEAU_PRES).c_str(), toString(PLATEAU_PRES, plat).c_str());
  write(elements_[PLATEAU_PRES].row, elements_[PLATEAU_PRES].col, buff);
}

void Display::writePEEP(const int& peep) {
  char buff[10];
  sprintf(buff, "  %4s=%2s", getLabel(PEEP_PRES).c_str(), toString(PEEP_PRES, peep).c_str());
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
      return (value > trigger_threshold_) ? String(value, 1) : "OFF";
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

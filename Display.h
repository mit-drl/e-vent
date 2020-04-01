#ifndef Display_h
#define Display_h

#include "Arduino.h"

#include <LiquidCrystal.h>


class Display {
public:
  // Make sure lcd is global
  Display(LiquidCrystal* lcd);

  // Call inside setup() instead of lcd.begin()
  void begin();

  // Write the top of the display
  void writeHeader();

  // Write volume as a percent of the max
  void writeVolume(const int& vol);

  // Beats per minute
  void writeBPM(const int& bpm);

  // Inhale/exhale ratio in format 1:ie
  void writeIEratio(const float& ie);

  // AC trigger pressure
  void writeACTrigger(const float& ac_trigger, const float& lower_threshold);

  // Peak pressure in cm of H2O
  void writePeakP(const int& peak);

  // Plateau pressure in cm of H2O
  void writePlateauP(const int& plat);

  // PEEP pressure in cm of H2O
  void writePEEP(const int& peep);

private:
  LiquidCrystal* lcd_;

  template <typename T>
  void write(const int& row, const int& col, const T& printable);
};


#endif

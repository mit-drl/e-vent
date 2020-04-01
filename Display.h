#ifndef Display_h
#define Display_h

#include "Arduino.h"

#include <LiquidCrystal.h>


namespace display {

static const int WIDTH = 20;
static const int HEIGHT = 4;
static const char* BLANK_LINE = "                    ";
static const int BLINK_PERIOD = 1000;  // milliseconds
static const float BLINK_ON_FRACTION = 0.5;


class TextAnimation {
public:
  void reset(const String& text = "");

  bool empty();

  const String& text();

  const String getLine();

private:
  String text_;
  unsigned long reset_time_;
};


class Display {
public:
  // Construct from lcd display
  // (make sure lcd is global)
  Display(LiquidCrystal* lcd);

  // Call inside setup() instead of lcd.begin()
  void begin();

  // Update alarm display
  // (for animation, e.g. blinking)
  void update();
  
  // Write arbitrary alarm in the header
  void setAlarm(const String& alarm);

  // Clear alarm
  void clearAlarm();

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
  TextAnimation animation_;

  // Write the top of the display
  void writeHeader();

  // Write printable starting at (row, col)
  template <typename T>
  void write(const int& row, const int& col, const T& printable);
};


}  // namespace display

#endif

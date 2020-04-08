#ifndef Display_h
#define Display_h

#include "Arduino.h"

#include <LiquidCrystal.h>


namespace display {

static const int kWidth = 20;  // Width of the display
static const int kHeight = 4;  // Height of the display


/**
 * TextAnimation
 * Handles the blinking of text in the display.
 */
class TextAnimation {
  const char* kBlankLine = "                    ";  // One blank line of display width
  const int kBlinkPeriod = 1000;        // Blinks this often in milliseconds
  const float kBlinkOnFraction = 0.5;   // Text shows for this fraction of the blinking period

public:

  // Reset the text of the animation
  void reset(const String& text = "");

  // Check if the animation text is empty
  bool empty();

  // Get the animation text
  const String& text();

  // Get the current string to display (text or blank line)
  const String getLine();

private:
  String text_;
  unsigned long reset_time_;
};


/**
 * Display
 * Handles writing ventilator-specific things to the display.
 */
class Display {
public:
  // Constructor, save a pointer to the (global) display object
  Display(LiquidCrystal* lcd);

  // Setup during arduino setup()
  void begin();

  // Update during arduino loop()
  void update();
  
  // Write arbitrary alarm in the header
  void writeAlarmText(const String& alarm);

  // Write volume in mL
  void writeVolume(const int& vol);

  // Beats per minute
  void writeBPM(const int& bpm);

  // Inhale/exhale ratio in format 1:ie
  void writeIEratio(const float& ie);

  // AC trigger pressure
  void writeACTrigger(const float& ac_trigger, const bool& ac_enabled);

  // Peak pressure in cm of H2O
  void writePeakP(const int& peak);

  // Plateau pressure in cm of H2O
  void writePlateauP(const int& plat);

  // PEEP pressure in cm of H2O
  void writePEEP(const int& peep);

  // This displays a patient icon
  void showPatientIcon(const int& row, const int& col);

  // This displays a time icon
  void showTimeIcon(const int& row, const int& col);

  // Hides icon
  void hideIcon(const int& row, const int& col);

private:
  LiquidCrystal* lcd_;
  TextAnimation animation_;

  // Write printable starting at (row, col)
  template <typename T>
  void write(const int& row, const int& col, const T& printable);
};


}  // namespace display

#endif

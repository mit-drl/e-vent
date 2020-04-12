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


enum DisplayKey {
  HEADER,
  VOLUME,
  BPM,
  IE_RATIO,
  AC_TRIGGER,
  PRES_LABEL,
  PEAK_PRES,
  PLATEAU_PRES,
  PEEP_PRES,
  NUM_KEYS
};


/**
 * Display
 * Handles writing ventilator-specific elements to the display.
 * Default format (without alarms) is as follows:
 *
 *              1111111111    
 *    01234567890123456789    
 *    ____________________ 
 * 0 |V=### mL   P(cmH2O):|
 * 1 |RR=##/min    peak=##|
 * 2 |I:E=1:#.#    plat=##|
 * 3 |AC=#.#cmH20  PEEP=##|
 *    ____________________ 
 */
class Display {

  struct Element {
    int row;
    int col;
    int width;  
  };

public:
  // Constructor, save a pointer to the (global) display object
  Display(LiquidCrystal* lcd): lcd_(lcd) {
    elements_[HEADER]       = Element{0, 0, 20};
    elements_[VOLUME]       = Element{0, 0, 11};
    elements_[BPM]          = Element{1, 0, 11};
    elements_[IE_RATIO]     = Element{2, 0, 11};
    elements_[AC_TRIGGER]   = Element{3, 0, 11};
    elements_[PRES_LABEL]   = Element{0, 11, 9};
    elements_[PEAK_PRES]    = Element{1, 11, 9};
    elements_[PLATEAU_PRES] = Element{2, 11, 9};
    elements_[PEEP_PRES]    = Element{3, 11, 9};
  }

  // Setup during arduino setup()
  void begin();

  // Update during arduino loop()
  void update();

  // Write printable corresponding to key'ed element
  void write(const DisplayKey& key, const String& printable);
  
  // Write arbitrary alarm in the header
  void writeAlarmText(const String& alarm);

  // Write volume in mL
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
  Element elements_[NUM_KEYS];

  // Write printable starting at (row, col)
  template <typename T>
  void write(const int& row, const int& col, const T& printable);
};


}  // namespace display

#endif

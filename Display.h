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
 * Display.h
 * Handles writing of E-Vent-specific content to the LCD.
 */

#ifndef Display_h
#define Display_h

#include "Arduino.h"
#include <LiquidCrystal.h>

#include "Utilities.h"


namespace display {

static const int kWidth = 20;  // Width of the display
static const int kHeight = 4;  // Height of the display


/**
 * TextAnimation
 * Handles the blinking of text in the display.
 */
class TextAnimation {
public:
  TextAnimation(const unsigned long& period, const float& on_fraction):
      pulse_(period, on_fraction) {}

  // Reset the text of the animation
  inline void reset(const String& text = "") { text_ = text; }

  // Check if the animation text is empty
  inline bool empty() { return text_.length() == 0; }

  // Get the animation text (original text passed to reset)
  inline const String& text() { return text_; }

  // Get the current string to display (empty string for blank)
  inline const String getLine() { return pulse_.read() ? text_ : ""; }

private:
  String text_;
  utils::Pulse pulse_;
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
 * 0 |AC=#.#     Pressure:|
 * 1 |TV=###       peak=##|
 * 2 |RR=##        plat=##|
 * 3 |IE=1:#.#     PEEP=##|
 *    ____________________ 
 */
class Display {

  struct Element {
    Element() = default;

    Element(const int& r, const int& c, const int& w, const String& l = ""):
        row(r), col(c), width(w), label(l) {
      while (blank.length() < width) blank += " ";
    }
    int row;
    int col;
    int width;  
    String label;
    String blank;
  };

public:
  // Constructor, save a pointer to the (global) display object
  Display(LiquidCrystal* lcd, const float& trigger_threshold):
      lcd_(lcd),
      trigger_threshold_(trigger_threshold),
      animation_(1000, 0.5) {
    elements_[HEADER]       = Element{0, 0, 20};
    elements_[VOLUME]       = Element{1, 0, 11, "TV"};
    elements_[BPM]          = Element{2, 0, 11, "RR"};
    elements_[IE_RATIO]     = Element{3, 0, 11, "IE"};
    elements_[AC_TRIGGER]   = Element{0, 0, 11, "AC"};
    elements_[PRES_LABEL]   = Element{0, 11, 9};
    elements_[PEAK_PRES]    = Element{1, 11, 9, "peak"};
    elements_[PLATEAU_PRES] = Element{2, 11, 9, "plat"};
    elements_[PEEP_PRES]    = Element{3, 11, 9, "PEEP"};
  }

  // Setup during arduino setup()
  void begin();

  // Update during arduino loop()
  void update();
  
  // Write arbitrary alarm in the header
  void setAlarmText(const String& alarm);

  // Write value corresponding to key'ed element
  template <typename T>
  void write(const DisplayKey& key, const T& value);

  // Write blank in the space corresponding to `key`
  void writeBlank(const DisplayKey& key);

  // Write current header
  void writeHeader();

  // Write volume in mL
  void writeVolume(const int& vol);

  // Beats per minute
  void writeBPM(const int& bpm);

  // Inhale/exhale ratio in format 1:ie
  void writeIEratio(const float& ie);

  // AC trigger pressure
  void writeACTrigger(const float& ac_trigger);

  // Label for pressure units
  void writePresLabel();

  // Peak pressure in cm of H2O
  void writePeakP(const int& peak);

  // Plateau pressure in cm of H2O
  void writePlateauP(const int& plat);

  // PEEP pressure in cm of H2O
  void writePEEP(const int& peep);

  // Convert value e.g. RR from numeric to string for displaying.
  template <typename T>
  String toString(const DisplayKey& key, const T& value) const;

  // Get label of given element (empty string for elements without label, e.g. HEADER)
  inline String getLabel(const DisplayKey& key) const { return elements_[key].label; };

private:
  LiquidCrystal* lcd_;
  const float trigger_threshold_;
  TextAnimation animation_;
  Element elements_[NUM_KEYS];

  // Write printable starting at (row, col)
  template <typename T>
  void write(const int& row, const int& col, const T& printable);

  inline bool alarmsON() const { return !animation_.empty(); }
};


// Instantiation of template methods
#define INSTANTIATE_WRITE(type) \
  template void Display::write(const DisplayKey& key, const type& value);
INSTANTIATE_WRITE(int)
INSTANTIATE_WRITE(float)
#undef INSTANTIATE


}  // namespace display

#endif

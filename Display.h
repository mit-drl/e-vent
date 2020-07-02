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

#include "Indicators.h"

namespace display {

static const int kWidth = 20;  // Width of the display
static const int kHeight = 4;  // Height of the display
static const int kHeaderLength = 19;  // Desired header length
static const int kFooterLength = 10;  // Desired footer length

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
  BREATHING,
  VOLUME,
  BPM,
  IE_RATIO,
  AC_TRIGGER,
  PRES_LABEL,
  PEAK_PRES,
  PLATEAU_PRES,
  PEEP_PRES,
  SNOOZE,
  BELL,
  FOOTER,
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
      animation_(1000, 0.5),
      animationfooter_(1000, 0.5),
      snoozecountdown_(0) {
    elements_[HEADER]       = Element{0, 0, kHeaderLength};
    elements_[BREATHING]    = Element{0, 19, 1};
    elements_[VOLUME]       = Element{1, 0, 4, "TV"};
    elements_[BPM]          = Element{1, 4, 3, "RR"};
    elements_[IE_RATIO]     = Element{1, 7, 5, "IE"};
    elements_[AC_TRIGGER]   = Element{1, 12, 5, "Ptr"};
    elements_[PEAK_PRES]    = Element{1, 17, 3, "peak"};
    elements_[PLATEAU_PRES] = Element{2, 17, 3, "plat"};
    elements_[PEEP_PRES]    = Element{3, 17, 3, "PEEP"};
    elements_[SNOOZE]       = Element{3, 1, 3};
    elements_[BELL]         = Element{3, 0, 1};
    elements_[FOOTER]       = Element{3, 7, kFooterLength};
  }

  // Setup during arduino setup()
  void begin();

  // Add Custom Characters to LCD Display during arduino setup()
  void addCustomCharacters();

  // Update during arduino loop()
  void update();
  
  // Set arbitrary alarm text
  void setAlarmText(const String& alarm);

  // Set unconfirmed knob alarm text
  void setUnconfirmedKnobAlarmText(const String& alarm);

  // Set snooze time text
  void setSnoozeText(const int& countdown);

  // Write value corresponding to key'ed element
  template <typename T>
  void write(const DisplayKey& key, const T& value);

  // Write blank in the space corresponding to `key`
  void writeBlank(const DisplayKey& key);

  // Write current header
  void writeHeader();

  // Write current footer
  void writeFooter();

  // Display snooze text
  void writeSnoozeTime();

  // Write volume in mL
  void writeVolume(const int& vol);

  // Beats per minute
  void writeBPM(const int& bpm);

  // Inhale/exhale ratio in format 1:ie
  void writeIEratio(const float& ie);

  // AC trigger pressure
  void writeACTrigger(const float& ac_trigger);

  // Peak pressure in cm of H2O
  void writePeakP(const int& peak);

  // Plateau pressure in cm of H2O
  void writePlateauP(const int& plat);

  // PEEP pressure in cm of H2O
  void writePEEP(const int& peep);

  // This displays a patient icon
  void showPatientIcon();

  // This displays a time icon
  void showTimeIcon();

  // Hide time-patient icon
  void hideTimePatientIcon();

  // Hides icon
  void hideIcon(const int& row, const int& col);

  // This displays the PIP icon
  void showPeakIcon(const int& row, const int& col);

  // This displays a plateau icon
  void showPlateauIcon(const int& row, const int& col);

  // This displays a PEEP icon
  void showPEEPIcon(const int& row, const int& col);

  // This displays a I: icon
  void showIEiIcon(const int& row, const int& col);

  // This displays 1: icon
  void showIE1Icon(const int& row, const int& col);

  // This displays a bell icon
  void showBellIcon();

  // Hide bell icon
  void hideBellIcon();

  // Convert value e.g. RR from numeric to string for displaying.
  template <typename T>
  String toString(const DisplayKey& key, const T& value) const;

  // Get label of given element (empty string for elements without label, e.g. HEADER)
  inline String getLabel(const DisplayKey& key) const { return elements_[key].label; };

private:
  LiquidCrystal* lcd_;
  const float trigger_threshold_;
  TextAnimation animation_;
  TextAnimation animationfooter_;
  unsigned long snoozecountdown_;
  Element elements_[NUM_KEYS];

  // Write printable starting at (row, col)
  template <typename T>
  void write(const int& row, const int& col, const T& printable);

  inline bool alarmsON() const { return (!animation_.empty() || !animationfooter_.empty()); }
};


// Instantiation of template methods
#define INSTANTIATE_WRITE(type) \
  template void Display::write(const DisplayKey& key, const type& value);
INSTANTIATE_WRITE(int)
INSTANTIATE_WRITE(float)
#undef INSTANTIATE_WRITE


}  // namespace display

#endif

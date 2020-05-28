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
 * Logging.h
 * Provides common interface for logging an arbitrary number of variables
 * to an SD card and/or to serial, e.g. the Serial monitor.
 */

#ifndef Logging_h
#define Logging_h

#include "Arduino.h"
#include <SD.h>
#include <SPI.h>


namespace logging {

/**
 * Var
 * A variable to be logged, with label and serialization methods.
 */
class Var {
public:
  Var() = default;

  // Set var label and pointer, and min digits and decimal digits for serialization
  template <typename T>
  Var(const String& label, T* var, const int& min_digits, const int& float_precision);

  // Get the variable label
  inline String label() const { return label_; }

  // Get a string representation of the variable pointed to
  String serialize() const;

private:
  String label_;
  int min_digits_;
  int float_precision_;

  union {
    bool* b;
    int* i;
    float* f;
    double* d;
  } var_;

  enum Type {
    BOOL,
    INT,
    FLOAT,
    DOUBLE
  } type_;

  String pad(String& s);

  void setPtr(const bool* var);

  void setPtr(const int* var);

  void setPtr(const float* var);

  void setPtr(const double* var);

  String serialize(bool* var) const;

  String serialize(int* var) const;

  String serialize(float* var) const;

  String serialize(double* var) const;
};


/**
 * Logger
 * Handles logging to serial or SD card.
 * 
 * Example usage:
 * 
 *    const bool log_to_serial = true;
 *    const bool log_to_card = true;
 *
 *    logging::Logger logger(log_to_serial, log_to_card);
 *    
 *    setup() {
 *      logger.addVar("var1_label", &var1);
 *      logger.addVar("var2_label", &var2);
 *      ...
 *      logger.addVar("varN_label", &varN);
 *
 *      const int sd_select_pin = 53;
 *      logger.begin(&Serial, sd_select_pin);
 *    }
 *    
 *    loop() {
 *      var1 = ...
 *      var2 = ...
 *      ...
 *      varN = ...
 *    
 *      logger.update();
 *    }
 */
class Logger {

  // Period for saving the file
  const unsigned long kSavePeriod = 1 * 1000UL;

  // Maximum number of variables to log
  static const int kMaxVars = 20;

public:
  // Set options
  Logger(bool log_to_serial, bool log_to_SD, 
         bool serial_labels = true, const String delim = "\t");

  // Add variable
  template <typename T>
  void addVar(const char var_name[], const T* var, 
              const int& min_digits = 1, const int& float_precision = 2);

  // Setup during arduino setup()
  // call after adding all the vars for the header to have them all
  void begin(const Stream* serial, const int& pin_select_SD);

  // Update during arduino loop()
  // Write all the variables to stream object and/or SD card
  void update();

private:
  // Options
  const bool log_to_serial_, log_to_SD_, serial_labels_;
  const String delim_;

  // Stream objects
  Stream* stream_;
  char filename_[12] = "DATA000.TXT";
  File file_;

  // Bookkeeping
  unsigned long last_save_ = 0;
  Var vars_[kMaxVars];
  int num_vars_ = 0;

  void makeFile();
};

// Instantiation of template methods
#define INSTANTIATE_ADDVAR(vartype) \
  template void Logger::addVar(const char var_name[], const vartype* var, \
                               const int& min_digits, const int& float_precision);
INSTANTIATE_ADDVAR(bool)
INSTANTIATE_ADDVAR(int)
INSTANTIATE_ADDVAR(float)
INSTANTIATE_ADDVAR(double)
#undef INSTANTIATE


}  // namespace logging

#endif

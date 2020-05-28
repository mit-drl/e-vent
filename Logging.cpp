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
 * Logging.cpp
 */

#include "Logging.h"


namespace logging {


/// Var

template <typename T>
Var::Var(const String& label, T* var, const int& min_digits, const int& float_precision):
    label_(label),
    min_digits_(min_digits),
    float_precision_(float_precision) {
  setPtr(var);
}


String Var::serialize() const {
  switch (type_) {
    case BOOL:
      return serialize(var_.b);
    case INT:
      return serialize(var_.i);
    case FLOAT:
      return serialize(var_.f);
    case DOUBLE:
      return serialize(var_.d);
  }
}

String Var::pad(String& s) {
  while (s.length() < min_digits_) {
    s = " " + s;
  }
  return s;
}

void Var::setPtr(const bool* var) {
  var_.b = var;
  type_ = BOOL;
}

void Var::setPtr(const int* var) {
  var_.i = var;
  type_ = INT;
}

void Var::setPtr(const float* var) {
  var_.f = var;
  type_ = FLOAT;
}

void Var::setPtr(const double* var) {
  var_.d = var;
  type_ = DOUBLE;
}

String Var::serialize(bool* var) const {
  String string_out((int)*var);
  return pad(string_out);
}

String Var::serialize(int* var) const {
  String string_out(*var);
  return pad(string_out);
}

String Var::serialize(float* var) const {
  String string_out(*var, float_precision_);
  return pad(string_out);
}

String Var::serialize(double* var) const {
  String string_out(*var, float_precision_);
  return pad(string_out);
}


/// Logger

Logger::Logger(bool log_to_serial, bool log_to_SD, 
               bool serial_labels, const String delim):
    log_to_serial_(log_to_serial),
    log_to_SD_(log_to_SD),
    serial_labels_(serial_labels),
    delim_(delim) {}

template <typename T>
void Logger::addVar(const char var_name[], const T* var, 
                    const int& min_digits, const int& float_precision) {
  vars_[num_vars_++] = Var(var_name, var, min_digits, float_precision);
}

void Logger::begin(const Stream* serial, const int& pin_select_SD) {
  stream_ = serial;

  if (log_to_SD_) {
    pinMode(pin_select_SD, OUTPUT);

    if (!SD.begin(pin_select_SD)) {
      return;
    }

    makeFile();
  }
}

void Logger::update() {
  if ((!log_to_serial_ && !log_to_SD_) || num_vars_ == 0) {
    return;
  }
  unsigned long time_now = millis();
  String line, line_with_labels;

  const bool need_line_without_labels = log_to_SD_ || (log_to_serial_ && !serial_labels_);
  const bool need_line_with_labels = log_to_serial_ && serial_labels_;

  for (int i = 0; i < num_vars_; i++) {
    String word = vars_[i].serialize();
    if (i != num_vars_ - 1) {
      word += delim_;
    }
    if (need_line_without_labels) {
      line += word;
    }
    if (need_line_with_labels) {
      line_with_labels += vars_[i].label() + ": " + word;
    }
  }
  
  if (log_to_serial_) {
    stream_->println(serial_labels_ ? line_with_labels : line);
  }

  if (log_to_SD_) {
    if (!file_) {
      file_ = SD.open(filename_, FILE_WRITE);
    }
    if (file_) {
      file_.println(line);
    }
    if (time_now - last_save_ > kSavePeriod) {
      file_.close();
      last_save_ = time_now;
    }
  }
}

void Logger::makeFile() {
  // Open file with number of last saved file
  int num;
  File number_file = SD.open("number.txt", FILE_READ);
  if (number_file) {
    num = number_file.parseInt();  
    number_file.close();
  }

  // Replace old number with new number
  SD.remove("number.txt");
  number_file = SD.open("number.txt", FILE_WRITE);
  if (number_file) {
    number_file.println(num + 1);
    number_file.close();
  }

  // Assign the number to the new file name
  snprintf(filename_, sizeof(filename_), "DATA%03d.TXT", num);
  
  // Print the header
  file_ = SD.open(filename_, FILE_WRITE);
  if (file_) {
    String line;
    for (int i = 0; i < num_vars_; i++) {
      line += vars_[i].label();
      if (i != num_vars_ - 1) {
        line += delim_;
      }
    }
    file_.println(line);
    file_.close();
    last_save_ = millis();
  }
}


}  // namespace logging


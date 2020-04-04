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
    case INT:
      return serialize(var_.i);
    case FLOAT:
      return serialize(var_.f);
    case DOUBLE:
      return serialize(var_.d);
  }
}



/// Logger

Logger::Logger(bool log_to_serial, bool log_to_SD, 
               bool serial_labels, const String delim):
    log_to_serial_(log_to_serial),
    log_to_SD_(log_to_SD),
    serial_labels_(serial_labels),
    delim_(delim) {}

void Logger::begin(const Stream& serial, const int& pin_select_SD) {
  serial_ = serial;

  if (log_to_SD_) {
    pinMode(pin_select_SD, OUTPUT);

    if (!SD.begin(pin_select_SD)) {
      if(serial_.available()) {
        serial_.println("SD card initialization failed!");
      }
      return;
    }

    if(serial_.available()) {
      serial_.println("SD card initialization done.");
    }

    makeFile();
  }
}

template <typename T>
void Logger::addVar(const char var_name[], T* var, 
                    const int& min_digits, const int& float_precision) {
  vars_[num_vars_++] = Var(var_name, var, min_digits, float_precision);
}

void Logger::log() {
  if ((!log_to_serial_ && !log_to_SD_) || num_vars_ == 0) {
    return;
  }

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
    serial_.println(serial_labels_ ? line_with_labels : line);
  }

  if (log_to_SD_) {
    File file = SD.open(filename_, FILE_WRITE);
    file.println(line);
    file.close();
  }
}

void Logger::makeFile() {
  // setup SD card data logger
  File number_file = SD.open("number.txt", FILE_READ);

  int num;
  if(number_file){
    num = number_file.parseInt();  

    number_file.close();
  }

  SD.remove("number.txt");
  
  number_file = SD.open("number.txt", FILE_WRITE);

  if(number_file){
    number_file.println(num+1);

    number_file.close();
  }

  snprintf(filename_, sizeof(filename_), "DATA%03d.TXT", num);

  if(serial_.available()) {
    serial_.print("DATA FILE NAME: ");
    serial_.println(filename_);
  }
  
  File dataFile = SD.open(filename_, FILE_WRITE);
  if (dataFile) {
    if(serial_.available()) {
      serial_.print("Writing to ");
      serial_.print(filename_);
      serial_.println("...");
    }
    dataFile.println("millis \tState \tMode \tPos \tVol \tBPM \tIE \tTin \tTex \tVin \tVex \tTrigSens \tPressure");
    dataFile.close();
    if(serial_.available()) {
      serial_.print("Writing to ");
      serial_.print(filename_);
      serial_.println("... done.");
    }
  } else {
    if(serial_.available()) {
      // if the file didn't open, print an error:
      serial_.print("error opening ");
      serial_.println(filename_);
    }
    // else throw an SD card error!
  }
}


}  // namespace logging


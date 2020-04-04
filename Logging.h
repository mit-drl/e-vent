#ifndef Logging_h
#define Logging_h

#include "Arduino.h"
#include <SD.h>
#include <SPI.h>


namespace logging {


class Var {
  static const int kMaxChars = 10;

public:
  Var() = default;

  template <typename T>
  Var(const String& label, T* var, const int& min_digits, const int& float_precision);

  inline String label() const { return label_; }

  String serialize() const;

private:
  String label_;
  int min_digits_;
  int float_precision_;

  union {
    int* i;
    float* f;
    double* d;
  } var_;

  enum Type {
    INT,
    FLOAT,
    DOUBLE
  } type_;

  void setPtr(int* var) {
    var_.i = var;
    type_ = INT;
  }

  String serialize(int* var) const {
    char buff[kMaxChars];
    sprintf(buff, "%*d", min_digits_, *var);
    return String(buff);
  }

  void setPtr(float* var) {
    var_.f = var;
    type_ = FLOAT;
  }

  String serialize(float* var) const {
    char buff[kMaxChars];
    dtostrf(*var, min_digits_, float_precision_, buff);
    return String(buff);
  }

  void setPtr(double* var) {
    var_.d = var;
    type_ = DOUBLE;
  }

  String serialize(double* var) const {
    char buff[kMaxChars];
    dtostrf(*var, min_digits_, float_precision_, buff);
    return String(buff);
  }
};


/**
 * Example usage:
 * 
 *    const bool log_to_serial = true;
 *    const bool log_to_card = true;
 *
 *    logging::Logger logger(log_to_serial, log_to_card);
 *    
 *    setup() {
 *      const int sd_select_pin = 53;
 *      logger.begin(Serial, sd_select_pin);
 *
 *      logger.addVar("var1_label", &var1);
 *      logger.addVar("var2_label", &var2);
 *      ...
 *      logger.addVar("varN_label", &varN);
 *    }
 *    
 *    loop() {
 *      var1 = ...
 *      var2 = ...
 *      ...
 *      varN = ...
 *    
 *      logger.log();
 *    }
 * 
 */
class Logger {
  static const int kMaxVars = 20;

public:
  Logger(bool log_to_serial, bool log_to_SD, 
         bool serial_labels = true, const String delim = "\t");

  void begin(const Stream& serial, const int& pin_select_SD);

  template <typename T>
  void addVar(const char var_name[], T* var, 
              const int& min_digits = 1, const int& float_precision = 2);

  void log();

private:
  // Options
  const bool log_to_serial_, log_to_SD_, serial_labels_;
  const String delim_;

  // Stream objects
  Stream& serial_;
  char filename_[12] = "DATA000.TXT";

  // Bookkeeping
  Var vars_[kMaxVars];
  int num_vars_ = 0;

  void makeFile();
};

// Instantiation of template methods
template 
void Logger::addVar(const char var_name[], int* var, 
                    const int& min_digits, const int& float_precision);
template
void Logger::addVar(const char var_name[], float* var, 
                    const int& min_digits, const int& float_precision);
template
void Logger::addVar(const char var_name[], double* var, 
                    const int& min_digits, const int& float_precision);


}  // namespace logging

#endif

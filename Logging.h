#ifndef Logging_h
#define Logging_h

#include "Arduino.h"
#include <SD.h>
#include <SPI.h>


namespace logging {

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
  Logger(bool log_to_serial, bool log_to_SD, const char delimiter = ',');

  void begin(const Stream& serial, const int& pin_select_SD);

  template <typename T>
  void addVar(const char[] var_name, T* var);

  void log();

private:
  const Stream& serial_;
  char* vars_[kMaxVars];
  // TODO finish implementation
};


}  // namespace alarms

#endif

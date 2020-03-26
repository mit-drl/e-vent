#ifndef Pressure_h
#define Pressure_h

#include "Arduino.h"

class Pressure {
public:
  Pressure(int pin): sense_pin_(pin), offset_(0.0), peak_(0.0) {}

  void calibrate() {
    offset_= read();
  }

  //Get pressure reading
  float read() {
    // read the voltage
    int V = analogRead(sense_pin_); 

    float Pmin = -100.0;   // pressure max in mbar
    float Pmax = 100.0;    // pressure min in mbar
    float Vmax = 1024;     // max voltage in range from analogRead
    float R = 32./37;      // Internal 32K resistor and external 5K resistor ratio
    // convert to pressure
    float pres = (10 * V/Vmax * R - 1) * (Pmax-Pmin)/8. + Pmin; //mmHg

    // convert to cmH20
    pres *= 1.01972;
    
    float calibrated_pressure = pres - offset_;
    peak_ = max(peak_, calibrated_pressure);
    return calibrated_pressure;
  }

  float get_peak_and_reset() {
    float peak = peak_;
    peak_ = 0.0;
    return peak;
  }

private:
  int sense_pin_;
  float offset_;
  float peak_;
};

#endif

#ifndef Pressure_h
#define Pressure_h

#include "Arduino.h"

class Pressure {
public:
  Pressure(int pin): sense_pin_(pin), peak_(0.0), current_(0.0) {}

  void setPlateau() {
    plateau_ = current_;
  }

  float getPlateau() {
    return plateau_;
  }

  //Get pressure reading
  void read() {
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

    // update peak
    peak_ = max(peak_, pres);

    current_ = pres;
  }

  float get() {
    return current_;
  }

  float get_peak_and_reset() {
    float peak = peak_;
    peak_ = 0.0;
    return peak;
  }

private:
  int sense_pin_;
  float peak_, plateau_, peep_;
  float current_;
};

#endif

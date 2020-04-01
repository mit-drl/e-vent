#ifndef Pressure_h
#define Pressure_h

#include "Arduino.h"

class Pressure {
public:
  Pressure(int pin): 
    sense_pin_(pin),
    current_(0.0),
    current_peak_(0.0),
    peak_(0.0),
    plateau_(0.0),
    peep_(0.0) {}

  //Get pressure reading
  void read() {
    // read the voltage
    int V = analogRead(sense_pin_); 

    float Pmin = -100.0;   // pressure max in mbar
    float Pmax = 100.0;    // pressure min in mbar
    float Vmax = 1024;     // max voltage in range from analogRead
    // convert to pressure
    float pres = (10 * V/Vmax - 1) * (Pmax-Pmin)/8. + Pmin; //mmHg

    // convert to cmH20
    pres *= 1.01972;

    // update peak
    current_peak_ = max(current_peak_, pres);

    current_ = pres;
  }

  float get() {
    return current_;
  }

  void set_peak_and_reset() {
    peak_ = current_peak_;
    current_peak_ = 0.0;
  }

  void set_plateau() {
    plateau_ = get();
  }

  void set_peep() {
    peep_ = get();
  }

  int peak() { return round(peak_); }
  int plateau() { return round(plateau_); }
  int peep() { return round(peep_); }

private:
  int sense_pin_;
  float current_;
  float current_peak_;
  float peak_, plateau_, peep_;
};

#endif

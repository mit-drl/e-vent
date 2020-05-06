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
 * Pressure.h
 * Calculates and stores the key pressure values of the breathing cycle.
 */

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

  const float& get() {
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

  const float& peak() { return peak_; }
  const float& plateau() { return plateau_; }
  const float& peep() { return peep_; }

private:
  int sense_pin_;
  float current_;
  float current_peak_;
  float peak_, plateau_, peep_;
};

#endif

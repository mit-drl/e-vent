#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.

#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

int pressPin = A0;
int pressReadPin = 7;
int curPress = 0;
float Pmin = -100.0;        // pressure max in mbar
float Pmax = 100.0;         // pressure min in mbar
int Vmax = 1024;            // max voltage in range from analogRead
bool isWritten = false;     // whether a pressure write is needed

Adafruit_7segment matrix = Adafruit_7segment();


//Get pressure reading
float getPressure() {
  // read the voltage
  int val = analogRead(pressPin); 
  
  // convert to pressure
  float pres = (Pmax-Pmin)*(val/(0.8*Vmax)-0.125) + Pmin;

  //convert to cmH20
  pres *= 1.01972;
  // convert to psi
  //pres /= 68.94757;
  
  return pres - 76.3;
}


void setup() {
  pinMode(pressReadPin, INPUT_PULLUP);
  analogReference(EXTERNAL);
  matrix.begin(0x70);
  matrix.writeDigitRaw(1, 0x80);
  matrix.writeDisplay(); 
}


void loop() {
  if(digitalRead(pressReadPin) == HIGH && !isWritten) {
    matrix.print(round(getPressure()*10)/10.0);
    matrix.writeDisplay();
    isWritten = true;
  }else if(digitalRead(pressReadPin) == LOW){
    isWritten = false;
  }
  delay(50);
}

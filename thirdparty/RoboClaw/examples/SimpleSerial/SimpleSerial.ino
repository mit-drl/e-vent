//Roboclaw simple serial example.  Set mode to 6.  Option to 4(38400 bps)
#include <SoftwareSerial.h>

//See limitations of Arduino SoftwareSerial

SoftwareSerial mySerial(10,11);

void setup() {
  mySerial.begin(38400);
}

void loop() {
  mySerial.write(1);
  mySerial.write(-127);
  delay(2000);
  mySerial.write(64);
  delay(1000);
  mySerial.write(127);
  mySerial.write(-1);
  delay(2000);
  mySerial.write(-64);
  delay(1000);
  mySerial.write(1);
  mySerial.write(-1);
  delay(2000);
  mySerial.write(0);
  delay(1000);
  mySerial.write(127);
  mySerial.write(-127);
  delay(2000);
  mySerial.write(0);
  delay(1000);
}

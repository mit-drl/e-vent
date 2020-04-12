//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10,11);	
RoboClaw roboclaw(&serial,10000);

#define address 0x80

void setup() {
  //Communciate with roboclaw at 38400bps
  roboclaw.begin(38400);
  roboclaw.ForwardMixed(address, 0);
  roboclaw.TurnRightMixed(address, 0);
}

void loop() {
  roboclaw.ForwardMixed(address, 64);
  delay(2000);
  roboclaw.BackwardMixed(address, 64);
  delay(2000);
  roboclaw.TurnRightMixed(address, 64);
  delay(2000);
  roboclaw.TurnLeftMixed(address, 64);
  delay(2000);
  roboclaw.ForwardMixed(address, 0);
  roboclaw.TurnRightMixed(address, 64);
  delay(2000);
  roboclaw.TurnLeftMixed(address, 64);
  delay(2000);
  roboclaw.TurnRightMixed(address, 0);
  delay(2000);
}

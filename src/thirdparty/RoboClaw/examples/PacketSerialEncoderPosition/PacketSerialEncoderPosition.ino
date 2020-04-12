//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10,11);	
RoboClaw roboclaw(&serial,10000);

#define address 0x80

//Display Encoder and Speed for Motor 1
void displayspeed(void)
{
  uint8_t status1,status2;
  bool valid1,valid2;
  int32_t enc1 = roboclaw.ReadEncM1(address, &status1, &valid1);
  int32_t speed1 = roboclaw.ReadSpeedM1(address, &status2, &valid2);
  
  if(valid1){
    Serial.print("Encoder1:");
    Serial.print(enc1,DEC);
    Serial.print(" ");
    Serial.print(status1,HEX);
    Serial.print(" ");
  }
  if(valid2){
    Serial.print("Speed1:");
    Serial.print(speed1,DEC);
    Serial.print(" ");
  }
  
  Serial.println();
}

//This is the first function arduino runs on reset/power up
void setup() {
  //Open Serial and roboclaw at 38400bps
  Serial.begin(57600);
  roboclaw.begin(38400);
  
  Serial.println("Starting...");
}

void loop() {
  roboclaw.SpeedAccelDeccelPositionM1(address,0,12000,0,11000,1);
  roboclaw.SpeedAccelDeccelPositionM1(address,0,12000,0,1000,0);
  roboclaw.SpeedAccelDeccelPositionM1(address,32000,12000,32000,11000,0);
  roboclaw.SpeedAccelDeccelPositionM1(address,32000,12000,32000,1000,0);
  long last = millis();
  while(millis()-last<5000){
    displayspeed();
    delay(50);
  }
}

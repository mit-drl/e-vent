//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10,11);	
RoboClaw roboclaw(&serial,10000);

#define address 0x80

//Velocity PID coefficients
#define Kp 1.0
#define Ki 0.5
#define Kd 0.25
#define qpps 44000

void setup() {
  //Open Serial and roboclaw serial ports
  Serial.begin(57600);
  roboclaw.begin(38400);
  
  //Set PID Coefficients
  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
  roboclaw.SetM2VelocityPID(address,Kd,Kp,Ki,qpps);  
}

void displayspeed(void)
{
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;
  
  int32_t enc1 = roboclaw.ReadEncM1(address, &status1, &valid1);
  int32_t enc2 = roboclaw.ReadEncM2(address, &status2, &valid2);
  int32_t speed1 = roboclaw.ReadSpeedM1(address, &status3, &valid3);
  int32_t speed2 = roboclaw.ReadSpeedM2(address, &status4, &valid4);

  Serial.print("Encoder1:");
  if(valid1){
    Serial.print(enc1,DEC);
    Serial.print(" ");
    Serial.print(status1,HEX);
    Serial.print(" ");
  }
  else{
	Serial.print("failed ");
  }
  Serial.print("Encoder2:");
  if(valid2){
    Serial.print(enc2,DEC);
    Serial.print(" ");
    Serial.print(status2,HEX);
    Serial.print(" ");
  }
  else{
	Serial.print("failed ");
  }
  Serial.print("Speed1:");
  if(valid3){
    Serial.print(speed1,DEC);
    Serial.print(" ");
  }
  else{
	Serial.print("failed ");
  }
  Serial.print("Speed2:");
  if(valid4){
    Serial.print(speed2,DEC);
    Serial.print(" ");
  }
  else{
	Serial.print("failed ");
  }
  Serial.println();
}

void loop() {
  uint8_t depth1,depth2;
  
  roboclaw.SpeedDistanceM1(address,12000,48000,1);
  roboclaw.SpeedDistanceM2(address,-12000,48000,1);
  do{
    displayspeed();
    roboclaw.ReadBuffers(address,depth1,depth2);
  }while(depth1!=0x80 && depth2!=0x80);	//Loop until distance command has completed
  
  delay(1000);

  roboclaw.SpeedDistanceM1(address,-12000,48000,1);
  roboclaw.SpeedDistanceM2(address,12000,48000,1);
  do{
    displayspeed();
    roboclaw.ReadBuffers(address,depth1,depth2);
  }while(depth1!=0x80 && depth2!=0x80);	//Loop until distance command has completed
  
  delay(1000);  //When no second command is given the motors will automatically slow down to 0 which takes 1 second

  roboclaw.SpeedDistanceM1(address,12000,48000,1);
  roboclaw.SpeedDistanceM2(address,-12000,48000,1);
  roboclaw.SpeedDistanceM1(address,-12000,48000,0);
  roboclaw.SpeedDistanceM2(address,12000,48000,0);
  roboclaw.SpeedDistanceM1(address,0,48000,0);
  roboclaw.SpeedDistanceM2(address,0,48000,0);
  do{
    displayspeed();
    roboclaw.ReadBuffers(address,depth1,depth2);
  }while(depth1!=0x80 && depth2!=0x80);	//Loop until distance command has completed
  
  delay(1000);
}

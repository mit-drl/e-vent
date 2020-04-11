/* Library functions
  RoboClaw(uint8_t receivePin, uint8_t transmitPin, uint32_t tout);

  bool ForwardM1(uint8_t address, uint8_t speed);
  bool BackwardM1(uint8_t address, uint8_t speed);
  bool SetMinVoltageMainBattery(uint8_t address, uint8_t voltage);
  bool SetMaxVoltageMainBattery(uint8_t address, uint8_t voltage);
  bool ForwardM2(uint8_t address, uint8_t speed);
  bool BackwardM2(uint8_t address, uint8_t speed);
  bool ForwardBackwardM1(uint8_t address, uint8_t speed);
  bool ForwardBackwardM2(uint8_t address, uint8_t speed);
  bool ForwardMixed(uint8_t address, uint8_t speed);
  bool BackwardMixed(uint8_t address, uint8_t speed);
  bool TurnRightMixed(uint8_t address, uint8_t speed);
  bool TurnLeftMixed(uint8_t address, uint8_t speed);
  bool ForwardBackwardMixed(uint8_t address, uint8_t speed);
  bool LeftRightMixed(uint8_t address, uint8_t speed);
  uint32_t ReadEncM1(uint8_t address, uint8_t *status=NULL,bool *valid=NULL);
  uint32_t ReadEncM2(uint8_t address, uint8_t *status=NULL,bool *valid=NULL);
  bool SetEncM1(uint8_t address, int32_t val);
  bool SetEncM2(uint8_t address, int32_t val);
  uint32_t ReadSpeedM1(uint8_t address, uint8_t *status=NULL,bool *valid=NULL);
  uint32_t ReadSpeedM2(uint8_t address, uint8_t *status=NULL,bool *valid=NULL);
  bool ResetEncoders(uint8_t address);
  bool ReadVersion(uint8_t address,char *version);
  uint16_t ReadMainBatteryVoltage(uint8_t address,bool *valid=NULL);
  uint16_t ReadLogicBatteryVoltage(uint8_t address,bool *valid=NULL);
  bool SetMinVoltageLogicBattery(uint8_t address, uint8_t voltage);
  bool SetMaxVoltageLogicBattery(uint8_t address, uint8_t voltage);
  bool SetM1VelocityPID(uint8_t address, float Kp, float Ki, float Kd, uint32_t qpps);
  bool SetM2VelocityPID(uint8_t address, float Kp, float Ki, float Kd, uint32_t qpps);
  uint32_t ReadISpeedM1(uint8_t address,uint8_t *status=NULL,bool *valid=NULL);
  uint32_t ReadISpeedM2(uint8_t address,uint8_t *status=NULL,bool *valid=NULL);
  bool DutyM1(uint8_t address, uint16_t duty);
  bool DutyM2(uint8_t address, uint16_t duty);
  bool DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2);
  bool SpeedM1(uint8_t address, uint32_t speed);
  bool SpeedM2(uint8_t address, uint32_t speed);
  bool SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2);
  bool SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed);
  bool SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed);
  bool SpeedAccelM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2);
  bool SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag=0);
  bool SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag=0);
  bool SpeedDistanceM1M2(uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag=0);
  bool SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag=0);
  bool SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag=0);
  bool SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag=0);
  bool ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2);
  bool ReadPWMs(uint8_t address, int16_t &pwm1, int16_t &pwm2);
  bool ReadCurrents(uint8_t address, int16_t &current1, int16_t &current2);
  bool SpeedAccelM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2);
  bool SpeedAccelDistanceM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag=0);
  bool DutyAccelM1(uint8_t address, uint16_t duty, uint32_t accel);
  bool DutyAccelM2(uint8_t address, uint16_t duty, uint32_t accel);
  bool DutyAccelM1M2(uint8_t address, uint16_t duty1, uint32_t accel1, uint16_t duty2, uint32_t accel2);
  bool ReadM1VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps);
  bool ReadM2VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps);
  bool SetMainVoltages(uint8_t address,uint16_t min,uint16_t max);
  bool SetLogicVoltages(uint8_t address,uint16_t min,uint16_t max);
  bool ReadMinMaxMainVoltages(uint8_t address,uint16_t &min,uint16_t &max);
  bool ReadMinMaxLogicVoltages(uint8_t address,uint16_t &min,uint16_t &max);
  bool SetM1PositionPID(uint8_t address,float kp,float ki,float kd,float kiMax,uint32_t deadzone,uint32_t min,uint32_t max);
  bool SetM2PositionPID(uint8_t address,float kp,float ki,float kd,float kiMax,uint32_t deadzone,uint32_t min,uint32_t max);
  bool ReadM1PositionPID(uint8_t address,float &Kp,float &Ki,float &Kd,float &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max);
  bool ReadM2PositionPID(uint8_t address,float &Kp,float &Ki,float &Kd,float &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max);
  bool SpeedAccelDeccelPositionM1(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag);
  bool SpeedAccelDeccelPositionM2(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag);
  bool SpeedAccelDeccelPositionM1M2(uint8_t address,uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag);
  bool SetM1DefaultAccel(uint8_t address, uint32_t accel);
  bool SetM2DefaultAccel(uint8_t address, uint32_t accel);
  bool SetPinFunctions(uint8_t address, uint8_t S3mode, uint8_t S4mode, uint8_t S5mode);
  bool GetPinFunctions(uint8_t address, uint8_t &S3mode, uint8_t &S4mode, uint8_t &S5mode);
  bool SetDeadBand(uint8_t address, uint8_t Min, uint8_t Max);
  bool GetDeadBand(uint8_t address, uint8_t &Min, uint8_t &Max);
  bool RestoreDefaults(uint8_t address);
  bool ReadTemp(uint8_t address, uint16_t &temp);
  bool ReadTemp2(uint8_t address, uint16_t &temp);
  uint16_t ReadError(uint8_t address,bool *valid=NULL);
  bool ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode);
  bool SetM1EncoderMode(uint8_t address,uint8_t mode);
  bool SetM2EncoderMode(uint8_t address,uint8_t mode);
  bool WriteNVM(uint8_t address);
  bool ReadNVM(uint8_t address);
  bool SetConfig(uint8_t address, uint16_t config);
  bool GetConfig(uint8_t address, uint16_t &config);
  bool SetM1MaxCurrent(uint8_t address,uint32_t max);
  bool SetM2MaxCurrent(uint8_t address,uint32_t max);
  bool ReadM1MaxCurrent(uint8_t address,uint32_t &max);
  bool ReadM2MaxCurrent(uint8_t address,uint32_t &max);
  bool SetPWMMode(uint8_t address, uint8_t mode);
  bool GetPWMMode(uint8_t address, uint8_t &mode);
*/

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

//Uncomment if Using Hardware Serial port
//RoboClaw roboclaw(&Serial,10000);

//Uncomment if using SoftwareSerial. See limitations of Arduino SoftwareSerial
//SoftwareSerial serial(10,11);	
//RoboClaw roboclaw(&serial,10000);

#define address 0x80

void setup() {
  //Communicate at 38400bps
  roboclaw.begin(38400);
}

void loop()
{
}

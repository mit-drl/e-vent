/* 
 *  Sneaky Ventilator Code ReSpin
 *  MIT Underground
 *  3/17/2020
 */

#include <Encoder.h>


/* ##########################################################################
 * ##########################################################################
 * Initializations
 * ##########################################################################
 * ##########################################################################
 */
//Encoder input pins, use 2 and 3 for high performance
Encoder motorEncoder(2, 3);

//PWM pins for motor control
int PWM_A_PIN = 6;
int PWM_B_PIN = 5;
int PWM_A_OUT = 0;
int PWM_B_OUT = 0;

//Analog pin assignments
int freqPin = A1;
int dispPin = A0;
int iePin = A2;
int pressPin = A5;

//Variables for position control loops
long motorPosition = 0;
int positionDeadband = 5;
//This is the position feedback loop gain
int kPos = 1;
//This is timeout for reaching a position command before giving up (ms)
int movementTimeout = 2000;

//Variables for Velocity (pwm between 0 and 255)
int maxPwmForward = 255;
int maxPwmBackward = 255;
int currentVelocity = 0;
int deadBand = 0;

//Variables for acceleration
int maxAcceleration = 10;
//Delay (ms) between each velocity increase
int rampUpDelay = 1;
int rampStep = 20;

//Medical Variables
//freq is breathing frequency in Hz
int baseTime = 2;
//disp is displacement (how far to squeeze)
int disp = 1;
int dispScale = 1;
//ie is in/ex ratio, like duty cycle
float ie = 1.0;

//Main loop variables
int tStart = 0;
int tStop = 0;
int t = 0;
int timeScale = 1;

//Available states
enum State {
  STARTUP_STATE,
  IDLE_STATE,
  BREATHE_IN_STATE,
  HOLD_IN_STATE,
  BREATHE_OUT_STATE,
  HOLD_OUT_STATE,
};

State STATE = STARTUP_STATE;





/*
 * ##########################################################################
 * ##########################################################################
 * Functions
 * ##########################################################################
 * ##########################################################################
 */




//Get pressure reading
float pressurePSI() {
  float Pmin = -100.0;        // pressure max in mbar
  float Pmax = 100.0;         // pressure min in mbar
  float vsupply = 5000.0;   // voltage in volts
  // read the voltage
  int val = analogRead(pressPin); 
  // conver to millivolts
  float mv = 4.88*val;
  //Serial.println(mv);
  // convert to pressure
  float pres = (Pmax-Pmin)*(mv-0.1*vsupply)/(0.8*vsupply) + Pmin;
  //float pres = Pmin + ((Pmax - Pmin) / 0.8*vsupply) * (mv - 0.1*vsupply);

  //convert to cmH20
  pres *= 1.01972;
  // convert to psi
  //pres /= 68.94757;
  
  return pres;
}


 

//setVelocity takes a desiredVelocity between -255 and 255
//The velocity will be constrained to fit between these values
//this function does no acceleration control or checks, it only sets pwm
void setVelocity(int desiredVelocity) {
  int motorSpeed = constrain(desiredVelocity, -255, 255);
  if (motorSpeed > deadBand) {
    // Activate PWM_A; shut off PWM_B
    // Map the motorSpeed command to between deadband and 512 over to 8 bit 0-255
    PWM_A_OUT = map(motorSpeed, deadBand, 512, 0, maxPwmForward);
    //Serial.print("Motor speed A pwm out: ");
    //Serial.println(PWM_A_OUT);
    analogWrite(PWM_A_PIN, PWM_A_OUT);
    analogWrite(PWM_B_PIN, 0);
  }
  else if (motorSpeed < -deadBand) {
    // Activate PWMB
    // Make sure other PWM (PWM A) is 0
    // Map the current sensor value
    // Note that we want the minimum pot value to be the highest duty cycle
    PWM_B_OUT = map(motorSpeed, -deadBand, -512, 0, maxPwmBackward);
    //Serial.print("Motor speed B pwm out: ");
    //Serial.println(PWM_B_OUT);
    analogWrite(PWM_A_PIN, 0);
    analogWrite(PWM_B_PIN, PWM_B_OUT);
  }
  else {
    // ALL PWMS 0
    analogWrite(PWM_A_PIN, 0);
    analogWrite(PWM_B_PIN, 0);
  } 
}





//setPosition is a blocking routine that tries to reach a desired position
//It handles acceleration and deceleration
void setPosition(long desiredPosition, int maxVelocity){
  //Start by finding an error
  motorPosition = motorEncoder.read();
  int positionError = desiredPosition - motorPosition;

  //If we need to move backwards, change vel and step to be neg
  int dirRampStep = rampStep;
  int dirMaxVelocity = maxVelocity;
  if (positionError < 0) {
    dirRampStep = -rampStep;
  }

  //Ramp up in velocity, see how far we go during it
  int beforeRampPosition = motorEncoder.read();
  //Do a velocity ramp-up
  for (int j = 0; abs(j) < dirMaxVelocity; j=j+dirRampStep) {
    setVelocity(j);
    delay(rampUpDelay);
    currentVelocity = j;
  }
  int afterRampPosition = motorEncoder.read();
  int rampDistance = afterRampPosition - beforeRampPosition;

  //Subtract the distance needed for deceleration
  long predictedDecelPosition = desiredPosition - rampDistance;
  
  //Keep checking position until we reach our slow-down location
  while (abs(motorPosition - predictedDecelPosition) > positionDeadband) {
    //Serial.println(motorPosition);
    motorPosition = motorEncoder.read();
    //Sanity check to quit if error is getting worse
    if (abs(desiredPosition - motorPosition) > abs(positionError)){break;}
  }

  //Velocity ramp down
  for (int j = currentVelocity; abs(j) > 2*rampStep; j=j-dirRampStep) {
    setVelocity(j);
    delay(rampUpDelay);
  }
  setVelocity(0);
}





/*
 * ##########################################################################
 * ##########################################################################
 * Setup and Main
 * ##########################################################################
 * ##########################################################################
 */


//Setup runs once when the device starts
void setup(){
  Serial.begin(9600);
  pinMode(PWM_A_PIN, OUTPUT);
  pinMode(PWM_B_PIN, OUTPUT);
}




//Loop runs continuously forever
//Breathing control happens here
void loop(){

  timeScale = analogRead(freqPin);
  ie = analogRead(iePin) / 128 ;
  //breathe out
  setPosition(0, 222);
  //hold out
  delay(baseTime*timeScale);

  //breathe in
  setPosition(disp*analogRead(dispPin), 222);
  //hold in
  delay(baseTime*timeScale*ie);
  Serial.println(pressurePSI());
}

#include <Encoder.h>
#include <PID_v1.h>

// Settings
////////////

bool DEBUG = false; // For logging
int maxPwm = 255; // Maximum for PWM is 255 but this can be set lower
int deadzone = 5; // The PMW deadzone to avoid squealing
int loopPeriod = 25; // The period (ms) of the control loop delay
int goalTol = 20; // The number of clicks near the goal that is considered as having arrived
int rampThresh = 50; // The number of clicks near the goal to transition from velocity to position ramp
int pauseTime = 250; // Time in ms to pause after inhalation
int slackCompensator = 20; // How much below zero to drive in the inhalation to compensate for slack in the drivetrain

// Pins
////////////

//PWM pins for motor control
int PWM_A_PIN = 6;
int PWM_B_PIN = 5;
int PWM_A_OUT = 0;
int PWM_B_OUT = 0;

//Pot pin assignments
int VOL_PIN = A0;
int BPM_PIN = A1;
int IE_PIN = A2;

// Tmp for communication with other board
int ARDUINO_PIN = 10;

Encoder motorEncoder(2, 3); //Encoder input pins, use 2 and 3 for high performance
unsigned long lastEncoderUpdate;

// Initialize Vars
////////////////////

//Define motor variables
double goalPosition, motorPosition, goalVelocity, motorVelocity, cmdVelocity, cmdAccel, cmdVoltage;

// Define waveform parameters
double Vin, Vex, Tin, Tex, Volume;

// Define pot mappings
float BPM_MIN = 10;
float BPM_MAX = 25;
float IE_MIN = 1;
float IE_MAX = 3;
float VOL_MIN = 100;
float VOL_MAX = 225;

//Create position and velocity PIDs
double posKp=0.75, posKi=0.1, posKd=0.01;
PID posPID(&motorPosition, &cmdVelocity, &goalPosition, posKp, posKi, posKd, DIRECT);
double velKp=100, velKi=30.0, velKd=0.0;
PID velPID(&motorVelocity, &cmdAccel, &goalVelocity, velKp, velKi, velKd, DIRECT);

//Setup States
enum States {DEBUG_STATE, PAUSE_STATE, EX_STATE, IN_STATE};
States state;
bool enteringState;
unsigned long stateTimer;

// Functions
////////////

// Set the current state in the state machine
void setState(States newState){
  enteringState = true;
  state = newState;
  stateTimer = millis();
}

// readEncoder reads the motor position from the encoder
void readEncoder(){
  int newPosition = motorEncoder.read(); //Units: Clicks
  int deltaT = millis()-lastEncoderUpdate; //Units ms
  motorVelocity = (newPosition - motorPosition)/deltaT; //Units: Clicks/ms (Future note: this may need filtering)
  motorPosition = newPosition;
  lastEncoderUpdate = millis();
  if(DEBUG){
    Serial.print("Pos\t");
    Serial.print(motorPosition);
    Serial.print("\tVel\t");
    Serial.println(motorVelocity);
  }
}

// readPots reads the pot values and sets the waveform parameters
void readPots(){
  Volume = map(analogRead(VOL_PIN), 0, 1024, VOL_MIN, VOL_MAX);
  float bpm = map(analogRead(BPM_PIN), 0, 1024, BPM_MIN, BPM_MAX);
  float ie = map(analogRead(IE_PIN), 0, 1024, IE_MIN*10, IE_MAX*10)/10.0; // Carry one decimal place

  float period = 60.0/bpm; // seconds in each period
  Tin = period / (1 + ie);
  Tex = period - Tin;
  Vin = Volume/(Tin*1000.0); // Velocity in clicks/ms
  Vex = Volume/(Tex*1000.0); // Velocity in clicks/ms
  
  if(DEBUG){
    Serial.print("Vol: ");
    Serial.print(Volume);
    Serial.print("\tBPM: ");
    Serial.print(bpm);
    Serial.print("\tIE: ");
    Serial.print(ie);
    Serial.print("\tTin: ");
    Serial.print(Tin);
    Serial.print("\tTex:");
    Serial.print(Tex);
    Serial.print("\tVin:");
    Serial.print(Vin);
    Serial.print("\tVex:");
    Serial.println(Vex);
  }
}

// setPwmVoltage takes a desiredVoltage between -255 and 255 and sets the PWM pins, values less than deadzone go to 0
void setVoltage(int desiredVoltage) {
  cmdVoltage = constrain(desiredVoltage, -maxPwm, maxPwm);
  if(abs(cmdVoltage) < deadzone) cmdVoltage = 0;
  analogWrite(PWM_A_PIN, max(0, cmdVoltage));
  analogWrite(PWM_B_PIN, max(0, -cmdVoltage)); 
  if(DEBUG){
    Serial.print("Cmd\t");
    Serial.println(cmdVoltage);
  }
}

// updatePositionPID sets a motor voltage based on the position PID
void updatePositionPID(){
  if(posPID.Compute()){
    setVoltage(cmdVelocity);
  }
}

// updateVelocityPID sets a motor voltage based on the velocity PID
void updateVelocityPID(){
  if(velPID.Compute()){
    setVoltage(cmdVoltage + cmdAccel);
  }
}

// goToPosition goes to a desired position at the given speed, switching from velocity to position control at rampThresh
void goToPosition(double pos, double vel, int rampThresh){
  if(abs(motorPosition - pos) < rampThresh){
    goalPosition = pos;
    updatePositionPID();
  }else{
    if(motorPosition < pos){
      goalVelocity = vel;
    }
    else {
      goalVelocity = -vel;
    }
    updateVelocityPID();
  }
}

void setup() {
  pinMode(PWM_A_PIN, OUTPUT);
  pinMode(PWM_B_PIN, OUTPUT);
  pinMode(ARDUINO_PIN, OUTPUT);

  //Initialize
  motorEncoder.write(0);
  lastEncoderUpdate = millis();
  motorPosition = 0;
  motorVelocity = 0;
  goalPosition = 0;
  posPID.SetOutputLimits(-maxPwm, maxPwm);
  velPID.SetOutputLimits(-maxPwm, maxPwm);
  posPID.SetMode(AUTOMATIC);
  velPID.SetMode(AUTOMATIC);
  setState(PAUSE_STATE);

  if(DEBUG){
    // setup serial coms
    Serial.begin(115200);
    setState(DEBUG_STATE);
  }
}

void loop() {
  if(DEBUG){
    if(Serial.available() > 0){
      setState(Serial.parseInt());
      while(Serial.available() > 0) Serial.read();
    }
    Serial.print("State:\t");
    Serial.println((int)state);
  }

  // All States
  delay(loopPeriod);
  readEncoder();
  readPots();
  
  if(state == DEBUG_STATE){
    setVoltage(0);
  }
  
  else if(state == PAUSE_STATE){
    //Entering
    if(enteringState){
      digitalWrite(ARDUINO_PIN, HIGH);
      enteringState = false;
    }
    if(millis()-stateTimer > pauseTime){
      digitalWrite(ARDUINO_PIN, LOW);
      setState(EX_STATE);
    }
  }
  
  else if(state == EX_STATE){
    goToPosition(Volume, Vex, rampThresh);
    if(millis()-stateTimer > Tex && abs(motorPosition - Volume) < goalTol) setState(IN_STATE);
  }
  
  else if(state == IN_STATE){
    goToPosition(0, Vin, rampThresh);
    if(millis()-stateTimer > Tin && abs(motorPosition) < goalTol) setState(PAUSE_STATE);
  }
}

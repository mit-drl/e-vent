enum States {DEBUG_STATE, IN_STATE, PAUSE_STATE, EX_STATE, PREHOME_STATE, HOMING_STATE};

#include <LiquidCrystal.h>
#include <RoboClaw.h>
#include "Display.h"
#include "Pressure.h"

// Settings
////////////

bool DEBUG = false; // For logging
int maxPwm = 255; // Maximum for PWM is 255 but this can be set lower
int loopPeriod = 25; // The period (ms) of the control loop delay
int pauseTime = 250; // Time in ms to pause after inhalation
double Vex = 600; // Velocity to exhale
double Vhome = 30; //The speed (0-255) in volts to use during homing
int goalTol = 20; // The tolerance to start stopping on reaching goal
int bagHome = 100; // The bag-specific position of the bag edge
int pauseHome = 500; // The pause time (ms) during homing to ensure stability

// Pins
////////////

int VOL_PIN = A0;
int BPM_PIN = A1;
int IE_PIN = A2;
int PRESS_POT_PIN = A3;
int PRESS_SENSE_PIN = A4;
int HOME_PIN = 4;
int ROBO_D0 = 2;
int ROBO_D1 = 3;

// Initialize Vars
////////////////////

// Define waveform parameters
double Vin, Tin, Tex, Volume;  // Vex is fixed in settings above

// Define pot mappings
float BPM_MIN = 10;
float BPM_MAX = 30;
float IE_MIN = 1;
float IE_MAX = 4;
float VOL_MIN = 150;
float VOL_MAX = 700; // 900; // For full 

//Setup States
States state;
bool enteringState;
unsigned long stateTimer;

// Roboclaw
SoftwareSerial serial(ROBO_D0, ROBO_D1); 
RoboClaw roboclaw(&serial,10000);
#define address 0x80
// auto-tuned PID values for PG188
//#define Kp 6.03917
//#define Ki 0.94777
//#define Kd 0.0
//#define qpps 3187
#define Kp 6.38650
#define Ki 1.07623
#define Kd 0.0
#define qpps 3000
int motorPosition = 0;

// position PID values for PG188
#define pKp 8.0
#define pKi 0.0
#define pKd 0.0
#define kiMax 10.0
#define deadzone 0
#define minPos 0
#define maxPos 1000

// LCD Screen
const int rs = 12, en = 11, d4 = 10, d5 = 9, d6 = 8, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
Display displ(&lcd);

// Pressure
Pressure pressure(PRESS_SENSE_PIN);

// Functions
////////////

// Set the current state in the state machine
void setState(States newState){
  enteringState = true;
  state = newState;
  stateTimer = millis();
}

// readPots reads the pot values and sets the waveform parameters
void readPots(){
  Volume = map(analogRead(VOL_PIN), 0, 1024, VOL_MIN, VOL_MAX);
  float bpm = map(analogRead(BPM_PIN), 0, 1024, BPM_MIN, BPM_MAX);
  float ie = map(analogRead(IE_PIN), 0, 1024, IE_MIN*10, IE_MAX*10)/10.0; // Carry one decimal place

  float period = 60.0/bpm; // seconds in each period
  Tin = period / (1 + ie);
  Tex = period - Tin;
  Vin = Volume/Tin; // Velocity in clicks/s

  displ.writeVolume(map(Volume, VOL_MIN, VOL_MAX, 0, 100));
  displ.writeBPM(bpm);
  displ.writeIEratio(ie);
  if(DEBUG){
    Serial.print("State: ");
    Serial.print(state);
    Serial.print("\tPos: ");
    Serial.print(motorPosition);
    Serial.print("\tVol: ");
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
    Serial.print(Vex);
    Serial.print("\tPressure:");
    Serial.print(pressure.get());
    Serial.println();
  }
}

int readEncoder() {
  uint8_t robot_status;
  bool valid;
  motorPosition = roboclaw.ReadEncM1(address, &robot_status, &valid);
  return valid;
}

// goToPosition goes to a desired position at the given speed,
void goToPosition(int pos, int vel){
  bool valid = readEncoder();

  int accel = 10000;
  int deccel = 10000;
  
  if(valid){
    roboclaw.SpeedAccelDeccelPositionM1(address,accel,vel,deccel,pos,1);
    if(DEBUG){
      Serial.print("CmdVel: ");
      Serial.print(vel);
      Serial.print("\tCmdDiff: ");
      Serial.println(pos);
    }
  }
  else{
    Serial.println("encoder not valid; goToPosition command not sent");
  }
}

void setup() {

  // wait 1 sec for the roboclaw to boot up
  delay(1000);
  
  //Initialize
  pinMode(HOME_PIN, INPUT_PULLUP); // Pull up the limit switch
  analogReference(EXTERNAL); // For the pressure and pots reading
  displ.begin();
  setState(PREHOME_STATE); // Initial state
  roboclaw.begin(38400); // Roboclaw
  roboclaw.SetM1MaxCurrent(address, 10000); // Current limit is 10A
  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps); // Set Velocity PID Coefficients
  roboclaw.SetM1PositionPID(address,pKp,pKi,pKd,kiMax,deadzone,minPos,maxPos); // Set Position PID Coefficients
  roboclaw.SetEncM1(address, 0); // Zero the encoder
  
  if(DEBUG){
    // setup serial coms
    Serial.begin(115200);
    setState(DEBUG_STATE);
  }
}

void loop() {
  if(DEBUG){
    if(Serial.available() > 0){
      setState((States) Serial.parseInt());
      while(Serial.available() > 0) Serial.read();
    }
  }

  // All States
  delay(loopPeriod);
  readPots();
  readEncoder();

  // Update display header
  displ.writeHeader();
  
  // read pressure every cycle to keep track of peak
  pressure.read();
  
  if(state == DEBUG_STATE){
    // Stop motor
    roboclaw.ForwardM1(address,0);
  }
  
  else if(state == IN_STATE){
    //Entering
    if(enteringState){
      // Consider changing PID tunings
      enteringState = false;
      goToPosition(Volume, Vin);
    }

    // Consider checking we reached the destination for fault detection
    if(millis()-stateTimer > Tin*1000){
      setState(PAUSE_STATE);
    }
  }
  
  else if(state == PAUSE_STATE){
    // Entering
    if(enteringState){
      enteringState = false;
    }
    if(millis()-stateTimer > pauseTime){
      pressure.set_plateau();
      setState(EX_STATE);
    }
  }
  
  else if(state == EX_STATE){
    //Entering
    if(enteringState){
      //consider changing PID tunings
      enteringState = false;
      goToPosition(0, Vex);
    }
      
    if(millis()-stateTimer > Tex*1000){
      pressure.set_peak_and_reset();
      pressure.set_peep();
      displ.writePeakP(pressure.peak());
      displ.writePEEP(pressure.peep());
      displ.writePlateauP(pressure.plateau());
      setState(IN_STATE);
    }
  }

  else if(state == PREHOME_STATE){
    //Entering
    if(enteringState){
      enteringState = false;
      //Consider displaying homing status on the screen
      roboclaw.BackwardM1(address, Vhome);
    }

    // Check status of limit switch
    if(digitalRead(HOME_PIN) == LOW) {
      setState(HOMING_STATE); 
    }

    // Consider a timeout to give up on homing
  }

  else if(state == HOMING_STATE){
    //Entering
    if(enteringState){
      enteringState = false;
      //Consider displaying homing status on the screen
      roboclaw.ForwardM1(address, Vhome);
    }
    
    if(digitalRead(HOME_PIN) == HIGH) {
      roboclaw.ForwardM1(address, 0);
      roboclaw.SetEncM1(address, 0); // Zero the encoder
      delay(pauseHome); // Wait for things to settle
      goToPosition(bagHome, 300); // Stop motor
      delay(pauseHome); // Wait for things to settle
      roboclaw.SetEncM1(address, 0); // Zero the encoder
      setState(IN_STATE); 
    }
    // Consider a timeout to give up on homing
  }
}

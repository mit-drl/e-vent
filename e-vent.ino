enum States {
  DEBUG_STATE,      // 0
  IN_STATE,         // 1
  HOLD_IN_STATE,    // 2
  EX_STATE,         // 3
  PEEP_PAUSE_STATE, // 4
  HOLD_EX_STATE,    // 5
  PREHOME_STATE,    // 6
  HOMING_STATE,     // 7
};

#include <LiquidCrystal.h>
#include <RoboClaw.h>

#include "Alarms.h"
#include "Display.h"
#include "Logging.h"
#include "Pressure.h"

// General Settings
////////////

bool DEBUG = false; // For controlling and displaying via serial
int maxPwm = 255; // Maximum for PWM is 255 but this can be set lower
float tLoopPeriod = 0.025; // The period (s) of the control loop
float tHoldInDuration = 0.25; // Duration (s) to pause after inhalation
float tMinPeepPause = 0.05; // Time (s) to pause after exhalation / before watching for an assisted inhalation
float tExMax = 1.00; // Maximum exhale timef
float Vhome = 300; // The speed (clicks/s) to use during homing
float voltHome = 30; // The speed (0-255) in volts to use during homing
int goalTol = 10; // The tolerance to start stopping on reaching goal
int bagHome = 100; // The bag-specific position of the bag edge
float tPauseHome = 2.0*bagHome/Vhome; // The pause time (s) during homing to ensure stability

// Assist Control Flags and Settings
bool ASSIST_CONTROL = false; // Enable assist control
bool patientTriggered = false;
float triggerSensitivity;  // Tunable via a potentiometer. Its range is [2 cmH2O to 5 cmH2O] lower than PEEP
bool DetectionWindow;
float DP; // Driving Pressure = Plateau - PEEP

// Pins
////////////
int VOL_PIN = A0;
int BPM_PIN = A1;
int IE_PIN = A2;
int PRESS_POT_PIN = A3;
int PRESS_SENSE_PIN = A4;
int HOME_PIN = 10;
const int BEEPER_PIN = 11;
const int SNOOZE_PIN = 42;
const int SD_SELECT = 53;

// Safety settings
////////////////////
float MAX_PRESSURE = 40;
float MIN_PLATEAU_PRESSURE = 5; // ?????

// Initialize Vars
////////////////////
// Define cycle parameters
float vIn, vEx, tIn, tHoldIn, tEx, tPeriod, Volume;
float tCycleTimer, tLoopTimer; // Timer starting at each breathing cycle, and each control loop iteration
bool tLoopBuffer; // The amount of time left at the end of each loop
float bpm;  // Respiratory rate
float ieRatio;  // Inhale/exhale ratio
float pressure;  // Latest pressure reading

// Durations
float tCycleDuration;   // Duration of each cycle
float tExDuration;      // tEx - tHoldIn
float tPeriodDuration;  // tPeriod - tEx

// Define pot mappings
float BPM_MIN = 10;
float BPM_MAX = 40;
float IE_MIN = 1;
float IE_MAX = 4;
float VOL_MIN = 150;
float VOL_MAX = 630; // 900; // For full 
float TRIGGERSENSITIVITY_MIN = 0;
float TRIGGERSENSITIVITY_MAX = 5;
float TRIGGER_LOWER_THRESHOLD = 2;
int ANALOG_PIN_MAX = 1023; // The maximum count on analog pins

// Bag Calibration for AMBU Adult bag
float VOL_SLOPE = 9.39;
float VOL_INT = -102.2;

//Setup States
States state;
bool enteringState;
float tStateTimer;

// Roboclaw
RoboClaw roboclaw(&Serial3, 10000);
#define address 0x80

// auto-tuned PID values for PG188
#define Kp 6.38650
#define Ki 1.07623
#define Kd 0.0
#define qpps 3000
int motorPosition = 0;

// position PID values for PG188
#define pKp 9.0
#define pKi 0.0
#define pKd 0.0
#define kiMax 10.0
#define deadzone 0
#define minPos 0
#define maxPos 1000

// LCD Screen
const int rs = 9, en = 8, d4 = 7, d5 = 6, d6 = 5, d7 = 4;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
display::Display displ(&lcd);

// Alarms
alarms::AlarmManager alarm(BEEPER_PIN, SNOOZE_PIN, &displ);

// Logger

logging::Logger logger(true,    // log_to_serial,
                       true,    // log_to_SD, 
                       false,    // serial_labels, 
                       ",");   // delim

// Pressure
Pressure pressureReader(PRESS_SENSE_PIN);

// TODO: move function definitions after loop() or to classes if they don't use global vars
// Functions
////////////

// Returns the current time in seconds
float now() {
  return millis()*1e-3;
}

// Set the current state in the state machine
void setState(States newState){
  enteringState = true;
  state = newState;
  tStateTimer = now();
}

// readPots reads the pot values and sets the waveform parameters
void readPots(){
  Volume = map(analogRead(VOL_PIN), 0, ANALOG_PIN_MAX, VOL_MIN, VOL_MAX);
  bpm = map(analogRead(BPM_PIN), 0, ANALOG_PIN_MAX, BPM_MIN, BPM_MAX);
  ieRatio = map(analogRead(IE_PIN), 0, ANALOG_PIN_MAX, IE_MIN*10, IE_MAX*10)/10.0; // Carry one decimal place
  triggerSensitivity = map(analogRead(PRESS_POT_PIN), 0, ANALOG_PIN_MAX, TRIGGERSENSITIVITY_MIN*100, TRIGGERSENSITIVITY_MAX*100)/100.0; //Carry two decimal places

  tPeriod = 60.0/bpm; // seconds in each breathing cycle period
  tHoldIn = tPeriod / (1 + ieRatio);
  tIn = tHoldIn - tHoldInDuration;
  tEx = min(tHoldIn + tExMax, tPeriod - tMinPeepPause);
  tExDuration = tEx - tHoldIn;  // For logging
  tPeriodDuration = tPeriod - tEx;  // For logging
  
  vIn = Volume/tIn; // Velocity in (clicks/s)
  vEx = Volume/(tEx - tHoldIn); // Velocity out (clicks/s)

  displ.writeVolume(max(0,map(Volume, VOL_MIN, VOL_MAX, 0, 100) * VOL_SLOPE + VOL_INT));
  displ.writeBPM(bpm);
  displ.writeIEratio(ieRatio);
  displ.writeACTrigger(triggerSensitivity, TRIGGER_LOWER_THRESHOLD);
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
      Serial.print("CmdVel: ");  // TODO remove
      Serial.print(vel);
      Serial.print("\tCmdDiff: ");
      Serial.println(pos);
    }
  }
  else{
    if(DEBUG) {
      Serial.println("encoder not valid; goToPosition command not sent");
    }
    // ELSE THROW AN ALARM
  }
}

// home switch
bool homeSwitchPressed() {
  return digitalRead(HOME_PIN) == LOW;
}

// check for errors
void checkErrors() {
  // pressure above max pressure
  alarm.highPressure(pressureReader.get() >= MAX_PRESSURE);

  // only worry about low pressure after homing
  alarm.lowPressure(state < 4 && pressureReader.plateau() <= MIN_PLATEAU_PRESSURE);

  if(DEBUG){ //TODO integrate these into the alarm system
    // TODO what to do with these alarms
    // check for roboclaw errors
    bool valid;
    uint32_t error_state = roboclaw.ReadError(address, &valid);
    if(valid){
      if (error_state == 0x0001) { // M1 OverCurrent Warning
        Serial.println("TURN OFF DEVICE");
      }
      else if (error_state == 0x0008) { // Temperature Error
        Serial.println("OVERHEATED");
      }
      else if (error_state == 0x0100){ // M1 Driver Fault
        Serial.println("RESTART DEVICE");
      }
      else if (error_state == 0x1000) { // Temperature Warning
        Serial.println("TEMP HIGH");
      }
    } else {
      Serial.println("RESTART DEVICE");
    }
  }
}

// Set up logger level and variables
void setupLogger() {
  logger.addVar("Time", &tCycleTimer);
  logger.addVar("tCycle", &tCycleDuration);
  logger.addVar("State", (int*)&state);
  logger.addVar("Mode", (int*)&patientTriggered);
  logger.addVar("Pos", &motorPosition);
  logger.addVar("Vol", &Volume);
  logger.addVar("BPM", &bpm);
  logger.addVar("IE", &ieRatio);
  logger.addVar("tIn", &tIn);
  logger.addVar("tHoldIn", &tHoldInDuration);
  logger.addVar("tEx", &tExDuration);
  logger.addVar("tHoldOut", &tPeriodDuration);
  logger.addVar("vIn", &vIn);
  logger.addVar("vEx", &vEx);
  logger.addVar("TrigSens", &triggerSensitivity);
  logger.addVar("Pressure", &pressure);
  logger.begin(&Serial, SD_SELECT);
}

///////////////////
////// Setup //////
///////////////////

void setup() {
  // setup serial coms
  Serial.begin(115200);
  while(!Serial);

  if(DEBUG){
    setState(DEBUG_STATE);
  }

  // wait 1 sec for the roboclaw to boot up
  delay(1000);
  
  //Initialize
  setupLogger();
  alarm.begin();
  pinMode(HOME_PIN, INPUT_PULLUP); // Pull up the limit switch
  displ.begin();
  setState(PREHOME_STATE); // Initial state
  roboclaw.begin(38400); // Roboclaw
  roboclaw.SetM1MaxCurrent(address, 10000); // Current limit is 10A
  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps); // Set Velocity PID Coefficients
  roboclaw.SetM1PositionPID(address,pKp,pKi,pKd,kiMax,deadzone,minPos,maxPos); // Set Position PID Coefficients
  roboclaw.SetEncM1(address, 0); // Zero the encoder
}

//////////////////
////// Loop //////
//////////////////

void loop() {
  if(DEBUG){
    if(Serial.available() > 0){
      setState((States) Serial.parseInt());
      while(Serial.available() > 0) Serial.read();
    }
  }

  // All States
  tLoopTimer = now(); // Start the loop timer
  readPots();
  readEncoder();
  pressureReader.read();
  checkErrors();
  alarm.update();
  displ.update();
  
  // State Machine
  if(state == DEBUG_STATE){
    // Stop motor
    roboclaw.ForwardM1(address, 0);
  }
  
  else if(state == IN_STATE){
    //Entering
    if(enteringState){
      // Consider changing PID tunings
      enteringState = false;
      const float tNow = now();
      tCycleDuration = tNow - tCycleTimer;  // For logging
      tCycleTimer = tNow; // The cycle begins at the start of inspiration
      goToPosition(Volume, vIn);
    }

    // Consider checking we reached the destination for fault detection
    // We need to figure out how to account for the PAUSE TIME
    if(now()-tCycleTimer > tIn){
      setState(HOLD_IN_STATE);
    }
  }
  
  else if(state == HOLD_IN_STATE){
    // Entering
    if(enteringState){
      enteringState = false;
    }
    if(now()-tCycleTimer > tHoldIn){
      pressureReader.set_plateau(); //Consider using some signal processing to better catch this value
      setState(EX_STATE);
    }
  }
  
  else if(state == EX_STATE){
    //Entering
    if(enteringState){
      //consider changing PID tunings
      enteringState = false;
      goToPosition(0, vEx);
    }

    // go to LISTEN_STATE 
    if(motorPosition < goalTol){
      setState(PEEP_PAUSE_STATE);
    }
  }

  else if(state == PEEP_PAUSE_STATE){
    // Entering
    if(enteringState){
      enteringState = false;
    }
    
    if(now()-tStateTimer > tMinPeepPause){
      pressureReader.set_peep();
      setState(HOLD_EX_STATE);
    }
  }

  else if(state == HOLD_EX_STATE){
    // Entering
    if(enteringState){
      enteringState = false;
    }

    // Check if patient triggers inhale
    patientTriggered = pressureReader.get() < (pressureReader.peep() - triggerSensitivity) 
        && triggerSensitivity > TRIGGER_LOWER_THRESHOLD;

    if( patientTriggered ||  now() - tCycleTimer > tPeriod) {
      pressureReader.set_peak_and_reset();
      displ.writePeakP(pressureReader.peak());
      displ.writePEEP(pressureReader.peep());
      displ.writePlateauP(pressureReader.plateau());
      setState(IN_STATE);

      // Consider if this is really necessary
      if(!patientTriggered) pressureReader.set_peep(); // Set peep again if time triggered
    }
  }

  else if(state == PREHOME_STATE){
    //Entering
    if(enteringState){
      enteringState = false;
      //Consider displaying homing status on the screen
      roboclaw.BackwardM1(address, voltHome);
    }

    // Check status of limit switch
    if(homeSwitchPressed()) {
      setState(HOMING_STATE); 
    }

    // Consider a timeout to give up on homing
  }

  else if(state == HOMING_STATE){
    //Entering
    if(enteringState){
      enteringState = false;
      //Consider displaying homing status on the screen
      roboclaw.ForwardM1(address, voltHome);
    }
    
    if(!homeSwitchPressed()) {
      roboclaw.ForwardM1(address, 0);
      roboclaw.SetEncM1(address, 0); // Zero the encoder
      delay(tPauseHome * 1000); // Wait for things to settle
      goToPosition(bagHome, Vhome); // Stop motor
      delay(tPauseHome * 1000); // Wait for things to settle
      roboclaw.SetEncM1(address, 0); // Zero the encoder
      setState(IN_STATE); 
    }
    // Consider a timeout to give up on homing
  }

  // Serialize all logged variables
  logger.update();

  // Add a delay if there's still time in the loop period
  tLoopBuffer = max(0, tLoopPeriod - tLoopTimer);
  delay(tLoopBuffer);
}


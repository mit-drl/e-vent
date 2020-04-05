enum States {
  DEBUG_STATE,    // 0
  IN_STATE,       // 1
  PAUSE_STATE,    // 2
  EX_STATE,       // 3
  EX_PAUSE_STATE, // 4
  LISTEN_STATE,   // 5 (listen for inhalation to assist)
  PREHOME_STATE,  // 6
  HOMING_STATE,   // 7
};

enum PastInhaleType {TIME_TRIGGERED, PATIENT_TRIGGERED};

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
int loopPeriod = 25; // The period (ms) of the control loop delay
int pauseTime = 250; // Time in ms to pause after inhalation
int exPauseTime = 50; // Time in ms to pause after exhalation / before watching for an assisted inhalation
double Vex = 600; // Velocity to exhale
double Vhome = 300; // The speed clicks per second to use during homing
double voltHome = 30; // The speed (0-255) in volts to use during homing
int goalTol = 20; // The tolerance to start stopping on reaching goal
int bagHome = 100; // The bag-specific position of the bag edge
int pauseHome = 2000*bagHome/Vhome; // The pause time (ms) during homing to ensure stability

// Assist Control Flags and Settings
bool ASSIST_CONTROL = false; // Enable assist control
PastInhaleType pastInhale;
float TriggerSensitivity;  // Tunable via a potentiometer. Its range is [2 cmH2O to 5 cmH2O] lower than PEEP
bool DetectionWindow;
float DP; // Driving Pressure = Plateau - PEEP
unsigned long exhale_time;

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

// Define waveform parameters
double Vin, Tin, Tex, Volume;  // Vex is fixed in settings above

// Define pot mappings
float BPM_MIN = 10;
float BPM_MAX = 30;
float IE_MIN = 1;
float IE_MAX = 4;
float VOL_MIN = 150;
float VOL_MAX = 630; // 900; // For full 
float TRIGGERSENSITIVITY_MIN = 0;
float TRIGGERSENSITIVITY_MAX = 5;

float TRIGGER_LOWER_THRESHOLD = 2;

// Bag Calibration for AMBU Adult bag
float VOL_SLOPE = 9.39;
float VOL_INT = -102.2;

//Setup States
States state;
bool enteringState;
unsigned long stateTimer;

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

logging::Logger logger(true,   // log_to_serial,
                       true,   // log_to_SD, 
                       true,   // serial_labels, 
                       "\t");   // delim

// Pressure
Pressure pressure(PRESS_SENSE_PIN);

// TODO: move function definitions after loop() or to classes if they don't use global vars
// Functions
////////////

// Set the current state in the state machine
void setState(States newState){
  enteringState = true;
  state = newState;
  stateTimer = millis();
}

// Set the type of last inhale
void setInhaleType(PastInhaleType aType){
  pastInhale = aType;
}

// Get the type of last inhale
PastInhaleType getInhaleType(){
  return (PastInhaleType) pastInhale;
}

// readPots reads the pot values and sets the waveform parameters
void readPots(){
  Volume = map(analogRead(VOL_PIN), 0, 1024, VOL_MIN, VOL_MAX);
  float bpm = map(analogRead(BPM_PIN), 0, 1024, BPM_MIN, BPM_MAX);
  float ie = map(analogRead(IE_PIN), 0, 1024, IE_MIN*10, IE_MAX*10)/10.0; // Carry one decimal place
  TriggerSensitivity = map(analogRead(PRESS_POT_PIN), 0, 1024, TRIGGERSENSITIVITY_MIN*100, TRIGGERSENSITIVITY_MAX*100)/100.0; //Carry two decimal places

  float period = 60.0/bpm; // seconds in each period
  Tin = period / (1 + ie);
  Tex = period - Tin;
  Vin = Volume/Tin; // Velocity in clicks/s

  displ.writeVolume(max(0,map(Volume, VOL_MIN, VOL_MAX, 0, 100) * VOL_SLOPE + VOL_INT));
  displ.writeBPM(bpm);
  displ.writeIEratio(ie);
  displ.writeACTrigger(TriggerSensitivity, TRIGGER_LOWER_THRESHOLD);
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
  alarm.highPressure(pressure.get() >= MAX_PRESSURE);

  // only worry about low pressure after homing
  alarm.lowPressure(state < 4 && pressure.plateau() <= MIN_PLATEAU_PRESSURE);

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
  logger.addVar("state", (int*)&state);
  // logger.addVar("inhaletype", &inhaletype);
  logger.addVar("motorPosition", &motorPosition, 2);
  logger.addVar("Volume", &Volume);
  // logger.addVar("bpm", &bpm);
  // logger.addVar("ie", &ie);
  logger.addVar("Tin", &Tin);
  logger.addVar("Tex", &Tex);
  logger.addVar("Vin", &Vin);
  logger.addVar("Vex", &Vex);
  logger.addVar("TriggerSensitivity", &TriggerSensitivity);
  // logger.addVar("pressure", pressure);
  logger.begin(&Serial, SD_SELECT);
}

///////////////////
////// Setup //////
///////////////////

void setup() {
  if(DEBUG){
    // setup serial coms
    Serial.begin(115200);
    while(!Serial);
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
  readPots();
  readEncoder();
  delay(loopPeriod);
  
  // read pressure every cycle to keep track of peak
  pressure.read();

  // Check errors and update alarm
  checkErrors();
  alarm.update();

  // Update display header
  displ.update();
  
  if(state == DEBUG_STATE){
    // Stop motor
    roboclaw.ForwardM1(address, 0);
  }
  
  else if(state == IN_STATE){
    //Entering
    if(enteringState){
      // Consider changing PID tunings
      enteringState = false;
      goToPosition(Volume, Vin);
    }

    // Consider checking we reached the destination for fault detection
    // We need to figure out how to account for the PAUSE TIME
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
      setInhaleType(TIME_TRIGGERED);
    }

    // go to LISTEN_STATE 
    if(motorPosition < goalTol){
      exhale_time = millis() - stateTimer;
      setState(EX_PAUSE_STATE);
    }

    // IF THERE IS A TIMEOUT for some reason
    // (the motor is not able to reach the 0 position),
    // just go straight to inhale?
    if(millis()-stateTimer > Tex*1000){
      pressure.set_peak_and_reset();
      pressure.set_peep();
      displ.writePeakP(pressure.peak());
      displ.writePEEP(pressure.peep());
      displ.writePlateauP(pressure.plateau());
      setState(IN_STATE);
    }
  }

  else if(state == EX_PAUSE_STATE){
    // Entering
    if(enteringState){
      enteringState = false;
    }
    
    if(millis()-stateTimer > exPauseTime){
      pressure.set_peep();
      setState(LISTEN_STATE);
    }
  }

  else if(state == LISTEN_STATE){
    // Entering
    if(enteringState){
      enteringState = false;
    }

    // PATIENT-triggered inhale
    if( pressure.get() < (pressure.peep() - TriggerSensitivity) && TriggerSensitivity > TRIGGER_LOWER_THRESHOLD ) {
      pressure.set_peak_and_reset();
      // note: PEEP is NOT set in this case;
      // we use the PEEP recorded in EX_PAUSE_STATE instead
      displ.writePeakP(pressure.peak());
      displ.writePEEP(pressure.peep());
      displ.writePlateauP(pressure.plateau());
      setState(IN_STATE);
      setInhaleType(PATIENT_TRIGGERED);
    }
    
    // TIME-triggered inhale
    if(millis()-stateTimer > Tex*1000 - exPauseTime - exhale_time){
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
      delay(pauseHome); // Wait for things to settle
      goToPosition(bagHome, Vhome); // Stop motor
      delay(pauseHome); // Wait for things to settle
      roboclaw.SetEncM1(address, 0); // Zero the encoder
      setState(IN_STATE); 
    }
    // Consider a timeout to give up on homing
  }
  logger.update();
}


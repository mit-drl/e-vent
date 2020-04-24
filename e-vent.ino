enum States {
  DEBUG_STATE,      // 0
  IN_STATE,         // 1
  HOLD_IN_STATE,    // 2
  EX_STATE,         // 3
  PEEP_PAUSE_STATE, // 4
  HOLD_EX_STATE,    // 5
  PREHOME_STATE,    // 6
  HOMING_STATE,     // 7
  OFF_STATE         // 8
};

#include <LiquidCrystal.h>
#include "src/thirdparty/RoboClaw/RoboClaw.h"

#include "Alarms.h"
#include "Buttons.h"
#include "Display.h"
#include "Input.h"
#include "Logging.h"
#include "Pressure.h"


// General Settings
////////////
bool DEBUG = false; // For controlling and displaying via serial TODO consolidate these two flags
const bool ENABLE_SERIAL_OVERRIDE = true; // For controlling via serial during automated testing
int maxPwm = 255; // Maximum for PWM is 255 but this can be set lower
float tLoopPeriod = 0.03; // The period (s) of the control loop
float tHoldInDuration = 0.25; // Duration (s) to pause after inhalation
float tMinPeepPause = 0.05; // Time (s) to pause after exhalation / before watching for an assisted inhalation
float tExMax = 1.00; // Maximum exhale timef
float Vhome = 300; // The speed (clicks/s) to use during homing
float voltHome = 30; // The speed (0-255) in volts to use during homing
float tPauseHome = 1.0; // The pause time (s) during homing to ensure stability
int goalTol = 10; // The tolerance to start stopping on reaching goal
float tAccel = 0.2; // Time for ramp up and ramp down (s)
int clearBag = 50;  // The value in clicks at which the fingers should retract to clear the bag

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
const int SNOOZE_PIN = 43;
const int CONFIRM_PIN = 41;
const int SD_SELECT = 53;
const int OFF_PIN = 45;
const int LED_ALARM_PIN = 12;

// Safety settings
////////////////////
const float MAX_PRESSURE = 40.0;
const float MIN_PLATEAU_PRESSURE = 5.0;
const float MAX_RESIST_PRESSURE = 2.0;
const float MIN_TIDAL_PRESSURE = 5.0;
const float VOLUME_ERROR_THRESH = 50.0;  // mL
const int MAX_MOTOR_CURRENT = 1000; // Max motor current

// Initialize Vars
////////////////////
// Define cycle parameters
unsigned long cycleCount = 0;
float vIn, vEx, tIn, tHoldIn, tEx, tPeriod, setVolume;
float tCycleTimer, tLoopTimer; // Timer starting at each breathing cycle, and each control loop iteration
float tLoopBuffer; // The amount of time left at the end of each loop
float bpm;  // Respiratory rate
float ieRatio;  // Inhale/exhale ratio

// Durations
float tCycleDuration;   // Duration of each cycle
float tExDuration;      // tEx - tHoldIn
float tExPauseDuration;  // tPeriod - tEx

// Define pot mappings
float BPM_MIN = 10;
float BPM_MAX = 40;
float IE_MIN = 1;
float IE_MAX = 4;
float VOL_MIN = 100;
float VOL_MAX = 800; // 900; // For full 
float TRIGGERSENSITIVITY_MIN = 0;
float TRIGGERSENSITIVITY_MAX = 5;
float TRIGGER_LOWER_THRESHOLD = 2;
int ANALOG_PIN_MAX = 1023; // The maximum count on analog pins

// Bag Calibration for AMBU Adult bag
const struct{float a, b, c;} COEFFS{1.29083271e-03, 4.72985182e-01, -7.35403067e+01};

// Calibration-dependent functions
/**
 * Converts motor position in ticks to volume in mL
 */
float ticks2volume(const float& vol_ticks) {
  return COEFFS.a * sq(vol_ticks) + COEFFS.b * vol_ticks + COEFFS.c;
}

/**
 * Converts volume in mL to motor position in ticks
 */
float volume2ticks(const float& vol_ml) {
  return (-COEFFS.b + sqrt(sq(COEFFS.b) -4 * COEFFS.a * (COEFFS.c - vol_ml))) / (2 * COEFFS.a);
}

//Setup States
States state;
bool enteringState;
float tStateTimer;

// Roboclaw
RoboClaw roboclaw(&Serial3, 10000);
#define address 0x80
int16_t motorCurrent;

// TODO can we refactor all these #define's into consts?
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
display::Display displ(&lcd, TRIGGER_LOWER_THRESHOLD);

// Alarms
alarms::AlarmManager alarm(BEEPER_PIN, SNOOZE_PIN, LED_ALARM_PIN, &displ, &cycleCount);

// Pressure
Pressure pressureReader(PRESS_SENSE_PIN);

// Buttons
buttons::PressHoldButton offButton(OFF_PIN, 2000);
buttons::DebouncedButton confirmButton(CONFIRM_PIN);

// Knobs
struct Knobs {
  void begin();
  void update();
  input::SafeKnob<int> volume     = input::SafeKnob<int>(&displ, display::VOLUME, CONFIRM_PIN, &alarm);
  input::SafeKnob<int> bpm        = input::SafeKnob<int>(&displ, display::BPM, CONFIRM_PIN, &alarm);
  input::SafeKnob<float> ie       = input::SafeKnob<float>(&displ, display::IE_RATIO, CONFIRM_PIN, &alarm);
  input::SafeKnob<float> trigger  = input::SafeKnob<float>(&displ, display::AC_TRIGGER, CONFIRM_PIN, &alarm);
} knobs;

// Serial active for testing and validation
bool serialActive = false;

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

// For reading of pots
int readVolume();         // Reads set volume (in mL) from the volume pot
int readBpm();            // Reads set bpm from the bpm pot
float readIeRatio();      // Reads set IE ratio from the IE pot
float readTriggerSens();  // Reads set trigger sensitivity from the trigger pot

// Reads user settings to set the waveform parameters
void readInput(){
  // Read knobs
  setVolume = knobs.volume.read();
  bpm = knobs.bpm.read();
  ieRatio = knobs.ie.read();
  triggerSensitivity = knobs.trigger.read();
}

// Calculates the waveform parameters from the user inputs
void calculateWaveform(){
  tPeriod = 60.0 / bpm; // seconds in each breathing cycle period
  tHoldIn = tPeriod / (1 + ieRatio);
  tIn = tHoldIn - tHoldInDuration;
  tEx = min(tHoldIn + tExMax, tPeriod - tMinPeepPause);
  tExDuration = tEx - tHoldIn;  // For logging
  tExPauseDuration = tPeriod - tEx;  // For logging
  
  vIn = (volume2ticks(setVolume) - clearBag) / (tIn - tAccel); // Velocity in (clicks/s)
  vEx = (volume2ticks(setVolume) - clearBag) / (tEx - tHoldIn - tAccel); // Velocity out (clicks/s)
}

int readEncoder() {
  uint8_t robot_status;
  bool valid;
  motorPosition = roboclaw.ReadEncM1(address, &robot_status, &valid);
  return valid;
}

bool readMotorCurrent() {
  int noSecondMotor;
  bool valid = roboclaw.ReadCurrents(address, motorCurrent, noSecondMotor);
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

// Check for errors and take appropriate action
void handleErrors() {
  // Pressure alarms
  const bool over_pressure = pressureReader.get() >= MAX_PRESSURE;
  alarm.highPressure(over_pressure);
  if(over_pressure) setState(EX_STATE);

  // These pressure alarms only make sense after homing 
  if (enteringState && state == IN_STATE) {
    alarm.badPlateau(pressureReader.peak() - pressureReader.plateau() > MAX_RESIST_PRESSURE);
    alarm.lowPressure(pressureReader.plateau() < MIN_PLATEAU_PRESSURE);
    alarm.noTidalPres(pressureReader.peak() - pressureReader.peep() < MIN_TIDAL_PRESSURE);
  }

  // Check if desired volume was reached
  if (enteringState && state == EX_STATE) {
    alarm.unmetVolume(setVolume - ticks2volume(motorPosition) > VOLUME_ERROR_THRESH);
  }

  // Check if maximum motor current was exceeded
  if(motorCurrent >= MAX_MOTOR_CURRENT){
    setState(EX_STATE);
    alarm.overCurrent(true);
  } else {
    alarm.overCurrent(false);
  }
  
  if(DEBUG){ //TODO integrate these into the alarm system
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


// Logger
logging::Logger logger(true,    // log_to_serial,
                       false,    // log_to_SD, 
                       false,    // serial_labels, 
                       ",\t");   // delim

// Set up logger variables
void setupLogger() {
  // logger.addVar("Time", &tLoopTimer);
  // logger.addVar("CycleStart", &tCycleTimer);
  // logger.addVar("Period", &tCycleDuration);
  // logger.addVar("tLoopBuffer", &tLoopBuffer, 6, 4);
  logger.addVar("State", (int*)&state);
  logger.addVar("Pos", &motorPosition, 3);
  // logger.addVar("Current", &motorCurrent, 3);
  logger.addVar("Pressure", &pressureReader.get(), 6);
  logger.addVar("Peep", &pressureReader.peep(), 6);
  // logger.addVar("Vol", &setVolume);
  // logger.addVar("BPM", &bpm);
  // logger.addVar("IE", &ieRatio);
  // logger.addVar("tIn", &tIn);
  // logger.addVar("tHoldIn", &tHoldInDuration);
  // logger.addVar("tEx", &tExDuration);
  // logger.addVar("tHoldOut", &tExPauseDuration);
  // logger.addVar("vIn", &vIn);
  // logger.addVar("vEx", &vEx);
  // logger.addVar("Mode", (int*)&patientTriggered);
  // logger.addVar("TrigSens", &triggerSensitivity);
  logger.addVar("HighPresAlarm", &alarm.getHighPressure());
  // begin called after all variables added to include them all in the header
  logger.begin(&Serial, SD_SELECT);
}

// Check the serial for automated testing commands
void readSerial() {
  while (Serial.available() > 0)
  {
    char first = Serial.read();
    int intVal;
    float floatVal;

    if(serialActive) {
        switch (first) {
            case '#':
                // Two hashes in a row toggle "is_active_"
                if (Serial.findUntil("#", "\n")) serialActive = false;
                break;

            case 'v':
                intVal = Serial.parseInt();
                if (VOL_MIN <= intVal && intVal <= VOL_MAX){
                  setVolume = intVal;
                  displ.writeVolume(setVolume);
                }
                break;

            case 'b':
                intVal = Serial.parseInt();
                if (BPM_MIN <= intVal && intVal <= BPM_MAX) {
                  bpm = intVal;
                  displ.writeBPM(bpm);
                }
                break;

            case 'e':
                floatVal = Serial.parseFloat();
                if (IE_MIN <= floatVal && floatVal <= IE_MAX) {
                  ieRatio = floatVal;
                  displ.writeIEratio(ieRatio);
                }
                break;

            case 't':
                floatVal = Serial.parseFloat();
                if (TRIGGERSENSITIVITY_MIN <= floatVal && floatVal <= TRIGGERSENSITIVITY_MAX) {
                  triggerSensitivity = floatVal;
                  displ.writeACTrigger(triggerSensitivity);
                }
                break;

            case 's':
                state = Serial.parseInt();
                break;
        }
    } else if (first == '#') {
        // Two hashes in a row toggle "is_active_"
        if (Serial.findUntil("#", "\n")) serialActive = true;
    }
  }
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
  offButton.begin();
  confirmButton.begin();
  knobs.begin();
  tCycleTimer = now();

  setState(PREHOME_STATE); // Initial state
  roboclaw.begin(38400); // Roboclaw
  roboclaw.SetM1MaxCurrent(address, 7000); // Current limit is 7A
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
  logger.update();  
  if (ENABLE_SERIAL_OVERRIDE) readSerial();
  if (!serialActive) {
    readInput();
    knobs.update();
  }
  calculateWaveform();
  readEncoder();
  readMotorCurrent();
  pressureReader.read();
  handleErrors();
  alarm.update();
  displ.update();
  offButton.update();

  if (offButton.wasHeld()) {
    goToPosition(clearBag, Vhome);
    setState(OFF_STATE);
    alarm.allOff();
  }
  
  // State Machine
  if(state == DEBUG_STATE){
    // Stop motor
    roboclaw.ForwardM1(address, 0);
  }

  else if (state == OFF_STATE) {
    alarm.turningOFF(now() - tStateTimer < 5.0);
    if (confirmButton.is_LOW()) {
      setState(PREHOME_STATE);
      alarm.turningOFF(false);
    }
  }
  
  else if(state == IN_STATE){
    //Entering
    if(enteringState){
      // Consider changing PID tunings
      enteringState = false;
      const float tNow = now();
      tCycleDuration = tNow - tCycleTimer;  // For logging
      tCycleTimer = tNow; // The cycle begins at the start of inspiration
      goToPosition(volume2ticks(setVolume), vIn);
      cycleCount++;
    }

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
      goToPosition(clearBag, vEx);
    }

    // go to LISTEN_STATE 
    if(motorPosition - clearBag < goalTol){
      setState(PEEP_PAUSE_STATE);
    }
  }

  else if(state == PEEP_PAUSE_STATE){
    // Entering
    if(enteringState){
      enteringState = false;
    }
    
    if(now() - tCycleTimer > tEx + tMinPeepPause){
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

    if(patientTriggered || now() - tCycleTimer > tPeriod) {
      if(!patientTriggered) pressureReader.set_peep(); // Set peep again if time triggered
      pressureReader.set_peak_and_reset();
      displ.writePeakP(round(pressureReader.peak()));
      displ.writePEEP(round(pressureReader.peep()));
      displ.writePlateauP(round(pressureReader.plateau()));
      setState(IN_STATE);
    }
  }

  else if(state == PREHOME_STATE){
    //Entering
    if(enteringState){
      enteringState = false;
      // TODO Consider displaying homing status on the screen
      roboclaw.BackwardM1(address, voltHome);
    }

    // Check status of limit switch
    if(homeSwitchPressed()) {
      setState(HOMING_STATE); 
    }

    // TODO Consider a timeout to give up on homing
  }

  else if(state == HOMING_STATE){
    //Entering
    if(enteringState){
      enteringState = false;
      // TODO Consider displaying homing status on the screen
      roboclaw.ForwardM1(address, voltHome);
    }
    
    if(!homeSwitchPressed()) {
      roboclaw.ForwardM1(address, 0);
      delay(tPauseHome * 1000); // Wait for things to settle
      roboclaw.SetEncM1(address, 0); // Zero the encoder
      setState(IN_STATE);
    }
    // TODO Consider a timeout to give up on homing
  }

  // Add a delay if there's still time in the loop period
  tLoopBuffer = max(0, tLoopTimer + tLoopPeriod - now());
  delay(tLoopBuffer*1000.0);
}


/////////////////
// Definitions //
/////////////////

void Knobs::begin() {
  volume.begin(&readVolume);
  bpm.begin(&readBpm);
  ie.begin(&readIeRatio);
  trigger.begin(&readTriggerSens);
}

void Knobs::update() {
  volume.update();
  bpm.update();
  ie.update();
  trigger.update();
}

int readVolume() {
  return map(analogRead(VOL_PIN), 0, ANALOG_PIN_MAX, VOL_MIN, VOL_MAX);
}

int readBpm() {
  return map(analogRead(BPM_PIN), 0, ANALOG_PIN_MAX, BPM_MIN, BPM_MAX); 
}

float readIeRatio() {
  return map(analogRead(IE_PIN), 0, ANALOG_PIN_MAX,
             IE_MIN*10, IE_MAX*10) / 10.0; // Carry one decimal place
}

float readTriggerSens() {
  return map(analogRead(PRESS_POT_PIN), 0, ANALOG_PIN_MAX,
             TRIGGERSENSITIVITY_MIN*100,
             TRIGGERSENSITIVITY_MAX*100) / 100.0; //Carry two decimal places
}

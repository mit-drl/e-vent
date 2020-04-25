#include "Constants.h"
#include "LiquidCrystal.h"
#include "src/thirdparty/RoboClaw/RoboClaw.h"
#include "Alarms.h"
#include "Buttons.h"
#include "Display.h"
#include "Input.h"
#include "Logging.h"
#include "Pressure.h"


// Initialize Vars
////////////////////

// Define cycle parameters
unsigned long cycleCount = 0;
float vIn, vEx, tIn, tHoldIn, tEx, tPeriod, setVolume;
float tCycleTimer, tLoopTimer; // Timer starting at each breathing cycle, and each control loop iteration
float tLoopBuffer; // The amount of time left at the end of each loop
float bpm;  // Respiratory rate
float ieRatio;  // Inhale/exhale ratio
float tCycleDuration;   // Duration of each cycle
float tExDuration;      // tEx - tHoldIn
float tExPauseDuration;  // tPeriod - tEx

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
int motorCurrent, motorPosition = 0;

// LCD Screen
LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D4_PIN, dLCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);
display::Display displ(&lcd, AC_MIN);

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

// Assist Control Vars
bool patientTriggered = false;
float triggerSensitivity;  // Tunable via a potentiometer. Its range is [2 cmH2O to 5 cmH2O] lower than PEEP
bool DetectionWindow;

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
  tIn = tHoldIn - HOLD_IN_DURATION;
  tEx = min(tHoldIn + MAX_EX_DURATION, tPeriod - MIN_PEEP_PAUSE);
  tExDuration = tEx - tHoldIn;  // For logging
  tExPauseDuration = tPeriod - tEx;  // For logging
  
  vIn = (volume2ticks(setVolume) - BAG_CLEAR_POS) / (tIn); // Velocity in (clicks/s)
  vEx = (volume2ticks(setVolume) - BAG_CLEAR_POS) / (tEx - tHoldIn); // Velocity out (clicks/s)
}

int readEncoder() {
  uint8_t robot_status;
  bool valid;
  motorPosition = roboclaw.ReadEncM1(ROBOCLAW_ADDR, &robot_status, &valid);
  return valid;
}

bool readMotorCurrent() {
  int noSecondMotor;
  bool valid = roboclaw.ReadCurrents(ROBOCLAW_ADDR, motorCurrent, noSecondMotor);
  return valid;
}

// goToPosition goes to a desired position at the given speed,
void goToPosition(int pos, int vel){
  bool valid = readEncoder();

  int accel = 10000;
  int deccel = 10000;
  
  if(valid){
    roboclaw.SpeedAccelDeccelPositionM1(ROBOCLAW_ADDR,accel,vel,deccel,pos,1);
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
    uint32_t error_state = roboclaw.ReadError(ROBOCLAW_ADDR, &valid);
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
  roboclaw.begin(ROBOCLAW_BAUD); // Roboclaw
  roboclaw.SetM1MaxCurrent(ROBOCLAW_ADDR, ROBOCLAW_MAX_CURRENT); // Current limit is 7A
  roboclaw.SetM1VelocityPID(ROBOCLAW_ADDR,VKD,VKP,VKI, QPPS); // Set Velocity PID Coefficients
  roboclaw.SetM1PositionPID(ROBOCLAW_ADDR,PKP,PKI,PKD,KI_MAX,DEADZONE,MIN_POS,MAX_POS); // Set Position PID Coefficients
  roboclaw.SetEncM1(ROBOCLAW_ADDR, 0); // Zero the encoder
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
  readInput();
  knobs.update();
  calculateWaveform();
  readEncoder();
  readMotorCurrent();
  pressureReader.read();
  handleErrors();
  alarm.update();
  displ.update();
  offButton.update();

  if (offButton.wasHeld()) {
    goToPosition(BAG_CLEAR_POS, HOMING_VEL);
    setState(OFF_STATE);
    alarm.allOff();
  }
  
  // State Machine
  if(state == DEBUG_STATE){
    // Stop motor
    roboclaw.ForwardM1(ROBOCLAW_ADDR, 0);
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
      pressureReader.set_plateau();
      setState(EX_STATE);
    }
  }
  
  else if(state == EX_STATE){
    //Entering
    if(enteringState){
      enteringState = false;
      goToPosition(BAG_CLEAR_POS, vEx);
    }

    // go to LISTEN_STATE 
    if(abs(motorPosition - BAG_CLEAR_POS) < BAG_CLEAR_TOL){
      setState(PEEP_PAUSE_STATE);
    }
  }

  else if(state == PEEP_PAUSE_STATE){
    // Entering
    if(enteringState){
      enteringState = false;
    }
    
    if(now() - tCycleTimer > tEx + MIN_PEEP_PAUSE){
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
        && triggerSensitivity > AC_MIN;

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
      roboclaw.BackwardM1(ROBOCLAW_ADDR, HOMING_VOLTS);
    }

    // Check status of limit switch
    if(homeSwitchPressed()) {
      setState(HOMING_STATE); 
    }
  }

  else if(state == HOMING_STATE){
    //Entering
    if(enteringState){
      enteringState = false;
      roboclaw.ForwardM1(ROBOCLAW_ADDR, HOMING_VOLTS);
    }
    
    if(!homeSwitchPressed()) {
      roboclaw.ForwardM1(ROBOCLAW_ADDR, 0);
      delay(HOMING_PAUSE * 1000); // Wait for things to settle
      roboclaw.SetEncM1(ROBOCLAW_ADDR, 0); // Zero the encoder
      setState(IN_STATE);
    }
  }

  // Add a delay if there's still time in the loop period
  tLoopBuffer = max(0, tLoopTimer + LOOP_PERIOD - now());
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
  return map(analogRead(AC_PIN), 0, ANALOG_PIN_MAX, 0, AC_MAX*100) / 100.0; //Carry two decimal places
}

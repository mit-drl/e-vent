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

#include <LiquidCrystal.h>
#include <RoboClaw.h>
#include <SPI.h>
#include <SD.h>

#include "Display.h"
#include "Alarms.h"
#include "Pressure.h"

// General Settings
////////////

bool LOGGER = true; // Data logger to a file on SD card
bool DEBUG = false; // For controlling and displaying via serial
int maxPwm = 255; // Maximum for PWM is 255 but this can be set lower
float tLoopPeriod = 0.025; // The period (s) of the control loop
float tHoldInDuration = 0.25; // Duration (s) to pause after inhalation
float tMinHoldOutDuration = 0.05; // Time (s) to pause after exhalation / before watching for an assisted inhalation
float tExMax = 1.00; // Maximum exhale timef
float Vhome = 300; // The speed (clicks/s) to use during homing
float voltHome = 30; // The speed (0-255) in volts to use during homing
int goalTol = 20; // The tolerance to start stopping on reaching goal
int bagHome = 100; // The bag-specific position of the bag edge
float tPauseHome = 2.0*bagHome/Vhome; // The pause time (s) during homing to ensure stability

// Assist Control Flags and Settings
bool ASSIST_CONTROL = false; // Enable assist control
bool patientTriggered;
float TriggerSensitivity;  // Tunable via a potentiometer. Its range is [2 cmH2O to 5 cmH2O] lower than PEEP
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

// Safety settings
////////////////////
float MAX_PRESSURE = 40;
float MIN_PLATEAU_PRESSURE = 5; // ?????

// Initialize Vars
////////////////////
// Define cycle parameters
float vIn, vEx, tIn, tHoldIn, tEx, tPeriod, Volume;
float tCycleTimer, tLoopTimer;

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
alarms::AlarmManager alarm(BEEPER_PIN, SNOOZE_PIN, &displ);

File dataFile;
char data_file_name[] = "DATA000.TXT";
const int chipSelect = 53;

// Pressure
Pressure pressure(PRESS_SENSE_PIN);

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
  float bpm = map(analogRead(BPM_PIN), 0, ANALOG_PIN_MAX, BPM_MIN, BPM_MAX);
  float ie = map(analogRead(IE_PIN), 0, ANALOG_PIN_MAX, IE_MIN*10, IE_MAX*10)/10.0; // Carry one decimal place
  TriggerSensitivity = map(analogRead(PRESS_POT_PIN), 0, ANALOG_PIN_MAX, TRIGGERSENSITIVITY_MIN*100, TRIGGERSENSITIVITY_MAX*100)/100.0; //Carry two decimal places

  //float vIn, vEx, tIn, tHoldIn, tEx, tPeriod, Volume;
  tPeriod = 60.0/bpm; // seconds in each breathing cycle period
  tHoldIn = tPeriod / (1 + ie);
  tIn = tHoldIn - tHoldInDuration;
  tEx = min(tHoldIn + tExMax, tPeriod - tMinHoldOutDuration);
  
  vIn = Volume/tIn; // Velocity in (clicks/s)
  vEx = Volume/(tEx - tHoldIn); // Velocity out (clicks/s)

  displ.writeVolume(max(0,map(Volume, VOL_MIN, VOL_MAX, 0, 100) * VOL_SLOPE + VOL_INT));
  displ.writeBPM(bpm);
  displ.writeIEratio(ie);
  displ.writeACTrigger(TriggerSensitivity, TRIGGER_LOWER_THRESHOLD);
  if(DEBUG){
    Serial.print("CycleTime: ");
    Serial.print(now() - tCycleTimer);    
    Serial.print("\tState: ");
    Serial.print(state);
    Serial.print("\tMode: "); // TIME or PATIENT triggered
    Serial.print((int) patientTriggered);
    Serial.print("\tPos: ");
    Serial.print(motorPosition);
    Serial.print("\tVol: ");
    Serial.print(Volume);
    Serial.print("\tBPM: ");
    Serial.print(bpm);
    Serial.print("\tIE: ");
    Serial.print(ie);
    Serial.print("\tIn: ");
    Serial.print(tIn);
    Serial.print("\tHoldIn: ");
    Serial.print(tHoldInDuration);
    Serial.print("\ttEx:");
    Serial.print(tEx - tHoldIn);
    Serial.print("\ttHoldOut:");
    Serial.print(tPeriod - tEx);
    Serial.print("\tvIn:");
    Serial.print(vIn);
    Serial.print("\tvEx:");
    Serial.print(vEx);
    Serial.print("\tTrigSens:");
    Serial.print(TriggerSensitivity);
    Serial.print("\tPressure:");
    Serial.print(pressure.get());
    Serial.println();
  }

  if(LOGGER){
    //Writing data to the SD Card
    dataFile = SD.open(data_file_name, FILE_WRITE);
    if (dataFile) {
      dataFile.print(millis()); dataFile.print("\t");
      dataFile.print(state); dataFile.print("\t");
      dataFile.print((int) patientTriggered); dataFile.print("\t");
      dataFile.print(motorPosition); dataFile.print("\t");
      dataFile.print(Volume); dataFile.print("\t");
      dataFile.print(bpm); dataFile.print("\t");
      dataFile.print(ie); dataFile.print("\t");
      dataFile.print(tIn); dataFile.print("\t");
      dataFile.print(tEx); dataFile.print("\t");
      dataFile.print(vIn); dataFile.print("\t");
      dataFile.print(vEx); dataFile.print("\t");
      dataFile.print(TriggerSensitivity); dataFile.print("\t");
      dataFile.print(pressure.get()); dataFile.println("\t");
      dataFile.close();
    } else {
      // if the file didn't open, print an error:
      if(DEBUG){
        Serial.print("error opening ");
        Serial.println(data_file_name);
      }
      // else we need to THROW AN SD ALARM
    }
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
    if(DEBUG) {
      Serial.println("encoder not valid; goToPosition command not sent");
    }
    // ELSE THROW AN ALARM
  }
}

void makeNewFile() {
  // setup SD card data logger
  pinMode(chipSelect, OUTPUT);
  if (!SD.begin(chipSelect)) {
    if(DEBUG) {
      Serial.println("SD card initialization failed!");
    }
    return;
  }

  if(DEBUG) {
    Serial.println("SD card initialization done.");
  }

  File number_file = SD.open("number.txt", FILE_READ);

  int num;
  if(number_file){
    num = number_file.parseInt();  

    number_file.close();
  }

  SD.remove("number.txt");
  
  number_file = SD.open("number.txt", FILE_WRITE);

  if(number_file){
    number_file.println(num+1);

    number_file.close();
  }

  snprintf(data_file_name, sizeof(data_file_name), "DATA%03d.TXT", num);

  if(DEBUG) {
    Serial.print("DATA FILE NAME: ");
    Serial.println(data_file_name);
  }
  
  dataFile = SD.open(data_file_name, FILE_WRITE);
  if (dataFile) {
    if(DEBUG) {
      Serial.print("Writing to ");
      Serial.print(data_file_name);
      Serial.println("...");
    }
    dataFile.println("millis \tState \tMode \tPos \tVol \tBPM \tIE \tTin \tTex \tVin \tVex \tTrigSens \tPressure");
    dataFile.close();
    if(DEBUG) {
      Serial.print("Writing to ");
      Serial.print(data_file_name);
      Serial.println("... done.");
    }
  } else {
    if(DEBUG) {
      // if the file didn't open, print an error:
      Serial.print("error opening ");
      Serial.println(data_file_name);
    }
    // else throw an SD card error!
  }
}

// home switch
bool homeSwitchPressed() {
  return digitalRead(HOME_PIN) == LOW;
}

// check for errors
void checkErrors() {
  // pressure above max pressure
  alarm.high_pressure(pressure.get() >= MAX_PRESSURE);

  // only worry about low pressure after homing
  alarm.low_pressure(state < 4 && pressure.plateau() <= MIN_PLATEAU_PRESSURE);

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


///////////////////
////// Setup //////
///////////////////

void setup() {

  // wait 1 sec for the roboclaw to boot up
  delay(1000);
  
  //Initialize
  alarm.begin();
  pinMode(HOME_PIN, INPUT_PULLUP); // Pull up the limit switch
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
    while(!Serial);
    setState(DEBUG_STATE);
  }

  if(LOGGER){
    makeNewFile();
  }
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
  pressure.read();
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
      tCycleTimer = now(); // The cycle begins at the start of inspiration
      goToPosition(Volume, vIn);
    }

    // Consider checking we reached the destination for fault detection
    // We need to figure out how to account for the PAUSE TIME
    if(now()-tCycleTimer > tIn){
      setState(PAUSE_STATE);
    }
  }
  
  else if(state == PAUSE_STATE){
    // Entering
    if(enteringState){
      enteringState = false;
    }
    if(now()-tCycleTimer > tHoldIn){
      pressure.set_plateau(); //Consider using some signal processing to better catch this value
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
      setState(EX_PAUSE_STATE);
    }
  }

  else if(state == EX_PAUSE_STATE){
    // Entering
    if(enteringState){
      enteringState = false;
    }
    
    if(now()-tStateTimer > tMinHoldOutDuration){
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
      patientTriggered = true;
    }
    
    // TIME-triggered inhale
    if(now() - tCycleTimer > tPeriod){
      pressure.set_peak_and_reset();
      pressure.set_peep();
      displ.writePeakP(pressure.peak());
      displ.writePEEP(pressure.peep());
      displ.writePlateauP(pressure.plateau());
      setState(IN_STATE);
      patientTriggered = false;
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

  // Add a delay if there's still time in the loop period
  delay(max(0, tLoopPeriod - tLoopTimer));
}

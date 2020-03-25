#include <LiquidCrystal.h>
#include <RoboClaw.h>
#include "Display.h"
#include <SPI.h>
#include <SD.h>

// Settings
////////////

bool LOGGER = true; // Data logger to a file on SD card
bool DEBUG = false; // For logging
int maxPwm = 255; // Maximum for PWM is 255 but this can be set lower
int loopPeriod = 25; // The period (ms) of the control loop delay
int pauseTime = 250; // Time in ms to pause after inhalation
double Vex = 600; //1000;  // Velocity to exhale
double rampTime = 0.5; // The time (s) the velocity profile takes to ramp up and down
double goalTol = 20;

// Pins
////////////

int VOL_PIN = A0;
int BPM_PIN = A1;
int IE_PIN = A2;
int PRESS_POT_PIN = A3;
int PRESS_SENSE_PIN = A4;
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
float VOL_MIN = 100;
float VOL_MAX = 600; // 900; // For full 

//Setup States
enum States {DEBUG_STATE, IN_STATE, PAUSE_STATE, EX_STATE};
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

// LCD Screen
double pressOffset = 0;
const int rs = 9, en = 8, d4 = 7, d5 = 6, d6 = 5, d7 = 4;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
Display displ(&lcd);

/* Data logger -- SD Card (Adafruit Breakout Board)
    Pin configurations per https://www.arduino.cc/en/reference/SPI
    CS  - pin 10
    DI  - pin 11
    DO  - pin 12
    CLK - pin 13
*/


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

  displ.writeVolume(100 * Volume/VOL_MAX);
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
    Serial.println(Vex);
  }

  if(LOGGER){
    // Writing data to the SD Card
    myFile = SD.open("ExpData.txt", FILE_WRITE);
    if (myFile) {
      myFile.println("------ NEW CLINICAL TRIAL DATA STARTS HERE ------"); 
      myFile.print(millis()); myFile.print("\t");
      myFile.print(state); myFile.print("\t");
      myFile.print(motorPosition); myFile.print("\t");
      myFile.print(Volume); myFile.print("\t");
      myFile.print(bpm); myFile.print("\t");
      myFile.print(ie); myFile.print("\t");
      myFile.print(Tin); myFile.print("\t");
      myFile.print(Tex); myFile.print("\t");
      myFile.print(Vin); myFile.print("\t");
      myFile.print(Vex); myFile.println("\t");
      myFile.close();
    } else {
      // if the file didn't open, print an error:
      Serial.println("error opening ExpData.txt");
    }
  }
}

//Get pressure reading
float readPressure() {
  // read the voltage
  int V = analogRead(PRESS_SENSE_PIN); 

  float Pmin = -100.0;        // pressure max in mbar
  float Pmax = 100.0;         // pressure min in mbar
  float Vmax = 1024;            // max voltage in range from analogRead
  float R = 32./37;      // Internal 32K resistor and external 5K resistor ratio
  // convert to pressure
  float pres = (10 * V/Vmax * R - 1) * (Pmax-Pmin)/8. + Pmin; //mmHg

  //convert to cmH20
  pres *= 1.01972;
  
  return pres - pressOffset;
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
  int diff = pos - motorPosition;

  if(diff < 0){
    vel = -vel; //want to reverse vel if you need to go backwards
    diff = abs(diff);
  }
  
  if(valid){
    roboclaw.SpeedDistanceM1(address,vel,diff, 1);
    Serial.print("CmdVel: ");
    Serial.print(vel);
    Serial.print("\tCmdDiff: ");
    Serial.println(diff);
  }
  else{
    Serial.println("encoder not valid; goToPosition command not sent");
  }
}

void setup() {

  // wait 1 sec for the roboclaw to boot up
  delay(1000);
  
  //Initialize
  analogReference(EXTERNAL); // For the pressure reading
  displ.begin();
  setState(IN_STATE); // Initial state
  roboclaw.begin(38400); // Roboclaw
  roboclaw.SetM1MaxCurrent(address, 10000); // Current limit is 10A
  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps); // Set PID Coefficients
  roboclaw.SetEncM1(address, 0); // Zero the encoder

  // Calibrate pressure sensor
  pressOffset = readPressure();
  
  if(DEBUG){
    // setup serial coms
    Serial.begin(115200);
    setState(DEBUG_STATE);
  }

  if(LOGGER){
    // setup SD card data logger
    pinMode(10, OUTPUT);
    if (!SD.begin(10)) {
      Serial.println("SD card initialization failed!");
      return;
    }
    Serial.println("SD card initialization done.");
    myFile = SD.open("ExpData.txt", FILE_WRITE);
    if (myFile) {
      Serial.print("Writing to ExpData.txt...");
      myFile.println("------ NEW CLINICAL TRIAL DATA STARTS HERE ------");
      myFile.println("millis \tState \tPos \tVol \tBPM \tIE \tTin \tTex \tVin \tVex");
      myFile.close();
      Serial.println("Writing to ExpData.txt... done.");
    } else {
      // if the file didn't open, print an error:
      Serial.println("error opening ExpData.txt");
    }
  }
}

void loop() {
  if(DEBUG){
    if(Serial.available() > 0){
      setState(Serial.parseInt());
      while(Serial.available() > 0) Serial.read();
    }
  }

  // All States
  delay(loopPeriod);
  readPots();
  readEncoder();
  
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
    
    if(millis()-stateTimer > Tin*1000 || abs(motorPosition - Volume) < goalTol)
      setState(PAUSE_STATE);
  }
  
  else if(state == PAUSE_STATE){
    // Entering
    if(enteringState){
      // Start the pressure averaging
      enteringState = false;
    }
    if(millis()-stateTimer > pauseTime){
      //Finish the pressure averaging
      displ.writePeakP(round(readPressure()));
      displ.writePlateauP(0);
      displ.writePEEP(0);
      
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

    if(abs(motorPosition) < goalTol)
      roboclaw.ForwardM1(address,0);
      
    if(millis()-stateTimer > Tex*1000)
      setState(IN_STATE);
  }
}

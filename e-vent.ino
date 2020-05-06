/**
 * MIT Emergency Ventilator Controller
 * 
 * MIT License:
 * 
 * Copyright (c) 2020 MIT
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * e-vent.ino
 * Main Arduino file.
 */

#include "LiquidCrystal.h"
#include "src/thirdparty/RoboClaw/RoboClaw.h"
#include "cpp_utils.h"  // Redefines macros min, max, abs, etc. into proper functions,
                        // should be included after third-party code, before E-Vent includes
#include "Alarms.h"
#include "Buttons.h"
#include "Constants.h"
#include "Display.h"
#include "Input.h"
#include "Logging.h"
#include "Pressure.h"


using namespace input;
using namespace utils;


/////////////////////
// Initialize Vars //
/////////////////////

// Cycle parameters
unsigned long cycleCount = 0;
float tCycleTimer;     // Absolute time (s) at start of each breathing cycle
float tIn;             // Calculated time (s) since tCycleTimer for end of IN_STATE
float tHoldIn;         // Calculated time (s) since tCycleTimer for end of HOLD_IN_STATE
float tEx;             // Calculated time (s) since tCycleTimer for end of EX_STATE
float tPeriod;         // Calculated time (s) since tCycleTimer for end of cycle
float tPeriodActual;   // Actual time (s) since tCycleTimer at end of cycle (for logging)
float tLoopTimer;      // Absolute time (s) at start of each control loop iteration
float tLoopBuffer;     // Amount of time (s) left at end of each loop

// States
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

// Logger
logging::Logger logger(true/*Serial*/, false/*SD*/, false/*labels*/, ",\t"/*delim*/);

// Knobs
struct Knobs {
  int volume();  // Tidal volume
  int bpm();     // Respiratory rate
  float ie();    // Inhale/exhale ratio
  float ac();    // Assist control trigger sensitivity
  SafeKnob<int> volume_ = SafeKnob<int>(&displ, display::VOLUME, CONFIRM_PIN, &alarm, VOL_RES);
  SafeKnob<int> bpm_ = SafeKnob<int>(&displ, display::BPM, CONFIRM_PIN, &alarm, BPM_RES);
  SafeKnob<float> ie_ = SafeKnob<float>(&displ, display::IE_RATIO, CONFIRM_PIN, &alarm, IE_RES);
  SafeKnob<float> ac_ = SafeKnob<float>(&displ, display::AC_TRIGGER, CONFIRM_PIN, &alarm, AC_RES);
  void begin();
  void update();
} knobs;

// Assist control
bool patientTriggered = false;


///////////////////////
// Declare Functions //
///////////////////////

// Set the current state in the state machine
void setState(States newState);

// Calculates the waveform parameters from the user inputs
void calculateWaveform();

// Check for errors and take appropriate action
void handleErrors();

// Set up logger variables
void setupLogger();


///////////////////
////// Setup //////
///////////////////

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  while(!Serial);

  if (DEBUG) {
    setState(DEBUG_STATE);
  } else {
    setState(PREHOME_STATE);  // Initial state
  }

  // Wait for the roboclaw to boot up
  delay(1000);
  
  //Initialize
  pinMode(HOME_PIN, INPUT_PULLUP);  // Pull up the limit switch
  setupLogger();
  alarm.begin();
  displ.begin();
  offButton.begin();
  confirmButton.begin();
  knobs.begin();
  tCycleTimer = now();

  roboclaw.begin(ROBOCLAW_BAUD);
  roboclaw.SetM1MaxCurrent(ROBOCLAW_ADDR, ROBOCLAW_MAX_CURRENT);
  roboclaw.SetM1VelocityPID(ROBOCLAW_ADDR, VKP, VKI, VKD, QPPS);
  roboclaw.SetM1PositionPID(ROBOCLAW_ADDR, PKP, PKI, PKD, KI_MAX, DEADZONE, MIN_POS, MAX_POS);
  roboclaw.SetEncM1(ROBOCLAW_ADDR, 0);  // Zero the encoder
}

//////////////////
////// Loop //////
//////////////////

void loop() {
  if (DEBUG) {
    if (Serial.available() > 0) {
      setState((States) Serial.parseInt());
      while(Serial.available() > 0) Serial.read();
    }
  }

  // All States
  tLoopTimer = now();  // Start the loop timer
  logger.update();
  knobs.update();
  calculateWaveform();
  readEncoder(roboclaw, motorPosition);  // TODO handle invalid reading
  readMotorCurrent(roboclaw, motorCurrent);
  pressureReader.read();
  handleErrors();
  alarm.update();
  displ.update();
  offButton.update();

  if (offButton.wasHeld()) {
    goToPositionByDur(roboclaw, BAG_CLEAR_POS, motorPosition, MAX_EX_DURATION);
    setState(OFF_STATE);
    alarm.allOff();
  }
  
  // State Machine
  switch (state) {

    case DEBUG_STATE:
      // Stop motor
      roboclaw.ForwardM1(ROBOCLAW_ADDR, 0);
      break;

    case OFF_STATE: 
      alarm.turningOFF(now() - tStateTimer < TURNING_OFF_DURATION);
      if (confirmButton.is_LOW()) {
        setState(PREHOME_STATE);
        alarm.turningOFF(false);
      }
      break;
  
    case IN_STATE:
      if (enteringState) {
        enteringState = false;
        const float tNow = now();
        tPeriodActual = tNow - tCycleTimer;
        tCycleTimer = tNow;  // The cycle begins at the start of inspiration
        goToPositionByDur(roboclaw, volume2ticks(knobs.volume()), motorPosition, tIn);
        cycleCount++;
      }

      if (now() - tCycleTimer > tIn) {
        setState(HOLD_IN_STATE);
      }
      break;
  
    case HOLD_IN_STATE:
      if (enteringState) {
        enteringState = false;
      }
      if (now() - tCycleTimer > tHoldIn) {
        pressureReader.set_plateau();
        setState(EX_STATE);
      }
      break;
  
    case EX_STATE:
      if (enteringState) {
        enteringState = false;
        goToPositionByDur(roboclaw, BAG_CLEAR_POS, motorPosition, tEx - (now() - tCycleTimer));
      }

      if (abs(motorPosition - BAG_CLEAR_POS) < BAG_CLEAR_TOL) {
        setState(PEEP_PAUSE_STATE);
      }
      break;

    case PEEP_PAUSE_STATE:
      if (enteringState) {
        enteringState = false;
      }
      
      if (now() - tCycleTimer > tEx + MIN_PEEP_PAUSE) {
        pressureReader.set_peep();
        
        setState(HOLD_EX_STATE);
      }
      break;

    case HOLD_EX_STATE:
      if (enteringState) {
        enteringState = false;
      }

      // Check if patient triggers inhale
      patientTriggered = pressureReader.get() < (pressureReader.peep() - knobs.ac()) 
          && knobs.ac() > AC_MIN;

      if (patientTriggered || now() - tCycleTimer > tPeriod) {
        if (!patientTriggered) pressureReader.set_peep();  // Set peep again if time triggered
        pressureReader.set_peak_and_reset();
        displ.writePeakP(round(pressureReader.peak()));
        displ.writePEEP(round(pressureReader.peep()));
        displ.writePlateauP(round(pressureReader.plateau()));
        setState(IN_STATE);
      }
      break;

    case PREHOME_STATE:
      if (enteringState) {
        enteringState = false;
        roboclaw.BackwardM1(ROBOCLAW_ADDR, HOMING_VOLTS);
      }

      if (homeSwitchPressed()) {
        setState(HOMING_STATE);
      }
      break;

    case HOMING_STATE:
      if (enteringState) {
        enteringState = false;
        roboclaw.ForwardM1(ROBOCLAW_ADDR, HOMING_VOLTS);
      }
      
      if (!homeSwitchPressed()) {
        roboclaw.ForwardM1(ROBOCLAW_ADDR, 0);
        delay(HOMING_PAUSE * 1000);  // Wait for things to settle
        roboclaw.SetEncM1(ROBOCLAW_ADDR, 0);  // Zero the encoder
        setState(IN_STATE);
      }
      break;
  }

  // Add a delay if there's still time in the loop period
  tLoopBuffer = max(0, tLoopTimer + LOOP_PERIOD - now());
  delay(tLoopBuffer*1000.0);
}


/////////////////
// Definitions //
/////////////////

void Knobs::begin() {
  volume_.begin(&readVolume);
  bpm_.begin(&readBpm);
  ie_.begin(&readIeRatio);
  ac_.begin(&readAc);
}

void Knobs::update() {
  volume_.update();
  bpm_.update();
  ie_.update();
  ac_.update();
}

inline int Knobs::volume() { return volume_.read(); }
inline int Knobs::bpm() { return bpm_.read(); }
inline float Knobs::ie() { return ie_.read(); }
inline float Knobs::ac() { return ac_.read(); }

void setState(States newState) {
  enteringState = true;
  state = newState;
  tStateTimer = now();
}

void calculateWaveform() {
  tPeriod = 60.0 / knobs.bpm();  // seconds in each breathing cycle period
  tHoldIn = tPeriod / (1 + knobs.ie());
  tIn = tHoldIn - HOLD_IN_DURATION;
  tEx = min(tHoldIn + MAX_EX_DURATION, tPeriod - MIN_PEEP_PAUSE);
}

void handleErrors() {
  // Pressure alarms
  const bool over_pressure = pressureReader.get() >= MAX_PRESSURE;
  alarm.highPressure(over_pressure);
  if (over_pressure) setState(EX_STATE);

  // These pressure alarms only make sense after homing 
  if (enteringState && state == IN_STATE) {
    alarm.badPlateau(pressureReader.peak() - pressureReader.plateau() > MAX_RESIST_PRESSURE);
    alarm.lowPressure(pressureReader.plateau() < MIN_PLATEAU_PRESSURE);
    alarm.noTidalPres(pressureReader.peak() - pressureReader.peep() < MIN_TIDAL_PRESSURE);
  }

  // Check if desired volume was reached
  if (enteringState && state == EX_STATE) {
    alarm.unmetVolume(knobs.volume() - ticks2volume(motorPosition) > VOLUME_ERROR_THRESH);
  }

  // Check if maximum motor current was exceeded
  if (motorCurrent >= MAX_MOTOR_CURRENT) {
    setState(EX_STATE);
    alarm.overCurrent(true);
  } else {
    alarm.overCurrent(false);
  }

  // Check if we've gotten stuck in EX_STATE (mechanical cycle didn't finsih)
  alarm.mechanicalFailure(state == EX_STATE && now() - tCycleTimer > tPeriod + MECHANICAL_TIMEOUT);
}

void setupLogger() {
  logger.addVar("Time", &tLoopTimer);
  logger.addVar("CycleStart", &tCycleTimer);
  logger.addVar("State", (int*)&state);
  logger.addVar("Pos", &motorPosition, 3);
  logger.addVar("Pressure", &pressureReader.get(), 6);
  // logger.addVar("Period", &tPeriodActual);
  // logger.addVar("tLoopBuffer", &tLoopBuffer, 6, 4);
  // logger.addVar("Current", &motorCurrent, 3);
  // logger.addVar("Peep", &pressureReader.peep(), 6);
  // logger.addVar("HighPresAlarm", &alarm.getHighPressure());
  // begin called after all variables added to include them all in the header
  logger.begin(&Serial, SD_SELECT);
}

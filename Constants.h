#ifndef Constants_h
#define Constants_h

// States
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

// Serial baud rate
const long SERIAL_BAUD_RATE = 115200;

// Flags
const bool DEBUG = false; // For controlling and displaying via serial
const bool ASSIST_CONTROL = false; // Enable assist control

// Timing Settings
const float LOOP_PERIOD = 0.03;       // The period (s) of the control loop
const float HOLD_IN_DURATION = 0.25;  // Duration (s) to pause after inhalation
const float MIN_PEEP_PAUSE = 0.05;    // Time (s) to pause after exhalation / before watching for an assisted inhalation
const float MAX_EX_DURATION = 1.00;   // Maximum exhale duration (s)

// Homing Settings
const float HOMING_VEL = 300; 	// The speed (clicks/s) to use during homing
const float HOMING_VOLTS = 30;	// The speed (0-255) in volts to use during homing
const float HOMING_PAUSE = 1.0; // The pause time (s) during homing to ensure stability
const int BAG_CLEAR_POS = 50;	  // The goal position (clicks) to retract to clear the bag
const int BAG_CLEAR_TOL = 10;	  // The tolerance (clicks) to consider clear of bag

// Pins
const int VOL_PIN = A0;
const int BPM_PIN = A1;
const int IE_PIN = A2;
const int AC_PIN = A3;
const int PRESS_SENSE_PIN = A4;
const int HOME_PIN = 10;
const int BEEPER_PIN = 11;
const int SNOOZE_PIN = 43;
const int CONFIRM_PIN = 41;
const int SD_SELECT = 53;
const int OFF_PIN = 45;
const int LED_ALARM_PIN = 12;
const int LCD_RS_PIN = 9;
const int LCD_EN_PIN = 8;
const int LCD_D4_PIN = 7;
const int dLCD_D5_PIN = 6;
const int LCD_D6_PIN = 5;
const int LCD_D7_PIN = 4;

// Control knob mappings
const float BPM_MIN = 10;
const float BPM_MAX = 40;
const float IE_MIN = 1;
const float IE_MAX = 4;
const float VOL_MIN = 100;
const float VOL_MAX = 800; 
const float AC_MIN = 2;
const float AC_MAX = 5;
const int ANALOG_PIN_MAX = 1023; // The maximum count on analog pins

// Bag Calibration for AMBU Adult bag
const struct {float a, b, c;} COEFFS{1.29083271e-03, 4.72985182e-01, -7.35403067e+01};

// Safety settings
const float MAX_PRESSURE = 40.0;		     // Trigger high pressure alarm
const float MIN_PLATEAU_PRESSURE = 5.0;	 // Trigger low pressure alarm
const float MAX_RESIST_PRESSURE = 2.0;	 // Trigger high-resistance notification
const float MIN_TIDAL_PRESSURE = 5.0;	   // Trigger no-tidal-pressure alarm
const float VOLUME_ERROR_THRESH = 50.0;	 // Trigger incomplete breath alarm
const int MAX_MOTOR_CURRENT = 1000;		   // Trigger mechanical failure alarm
const float TURNING_OFF_DURATION = 5.0;  // Turning-off alarm is on for this duration (s)

// PID values for auto-tuned for PG188
const float VKP = 6.38650;
const float VKI = 1.07623;
const float VKD = 0.0;
const unsigned long QPPS = 3000;
const float PKP = 9.0;
const float PKI= 0.0;
const float PKD = 0.0;
const unsigned long KI_MAX = 10;
const unsigned long DEADZONE = 0;
const unsigned long MIN_POS = 0;
const unsigned long MAX_POS = 1000;

// Roboclaw
const unsigned int ROBOCLAW_ADDR = 0x80;
const long ROBOCLAW_BAUD = 38400;
const unsigned long ROBOCLAW_MAX_CURRENT = 7000;

#endif

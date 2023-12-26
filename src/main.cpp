#include <Arduino.h>
//#include <stdint.h>
//#include <HardwareSerial.h>
//#include <stdlib.h>
#include "Keyboard.h"
#include <Joystick.h>

enum joystickAxis {
  X_AXIS,
  Y_AXIS,
  Z_AXIS,
  RX_AXIS,
  RY_AXIS,
  RZ_AXIS,
  RUDDER,
  THROTTLE,
  ACCELERATOR,
  BRAKE,
  STEERING
};

#define JOYSTICK_COUNT 2
#define kTELEOP_IX 0
#define kAUTO_IX 1
#define kKEYBOARD_IX 2 // not used in this version

// joystick layout
uint8_t kBUTTON_CNT[JOYSTICK_COUNT] = {4, 12};
uint8_t kPOV_CNT[JOYSTICK_COUNT] = {1, 2};
boolean kIS_X_AXIS[JOYSTICK_COUNT] = {true, false};
boolean kIS_Y_AXIS[JOYSTICK_COUNT] = {true, false};
boolean kIS_Z_AXIS[JOYSTICK_COUNT] = {true, false};
boolean kIS_X_ROTATION[JOYSTICK_COUNT] = {false, false};
boolean kIS_Y_ROTATION[JOYSTICK_COUNT] = {false, false};
boolean kIS_Z_ROTATION[JOYSTICK_COUNT] = {false, false};
boolean kIS_RUDDER[JOYSTICK_COUNT] = {false, false};
boolean kIS_THROTTLE[JOYSTICK_COUNT] = {false, false};
boolean kIS_ACCELRATOR[JOYSTICK_COUNT] = {false, false};
boolean kIS_BRAKE[JOYSTICK_COUNT] = {false, false};
boolean kIS_STEERING[JOYSTICK_COUNT] = {false, false};

Joystick_ Joystick[JOYSTICK_COUNT] = {
  Joystick_(0x03, JOYSTICK_TYPE_JOYSTICK,  kBUTTON_CNT[kTELEOP_IX], kPOV_CNT[kTELEOP_IX], 
        kIS_X_AXIS[kTELEOP_IX], kIS_Y_AXIS[kTELEOP_IX], kIS_Z_AXIS[kTELEOP_IX], 
        kIS_X_ROTATION[kTELEOP_IX], kIS_Y_ROTATION[kTELEOP_IX], kIS_Z_ROTATION[kTELEOP_IX],
        kIS_RUDDER[kTELEOP_IX], kIS_THROTTLE[kTELEOP_IX],
        kIS_ACCELRATOR[kTELEOP_IX], kIS_BRAKE[kTELEOP_IX], kIS_STEERING[kTELEOP_IX]),
  Joystick_(0x04, JOYSTICK_TYPE_JOYSTICK,  kBUTTON_CNT[kAUTO_IX], kPOV_CNT[kAUTO_IX], 
        kIS_X_AXIS[kAUTO_IX], kIS_Y_AXIS[kAUTO_IX], kIS_Z_AXIS[kAUTO_IX], 
        kIS_X_ROTATION[kAUTO_IX], kIS_Y_ROTATION[kAUTO_IX], kIS_Z_ROTATION[kAUTO_IX],
        kIS_RUDDER[kAUTO_IX], kIS_THROTTLE[kAUTO_IX],
        kIS_ACCELRATOR[kAUTO_IX], kIS_BRAKE[kAUTO_IX], kIS_STEERING[kAUTO_IX]) 
};

// Number of complete scans for which a key transition is ignored 
//    after the first, to reduce contact bounce. There's no obvious
//    way to calculate this value, and it will vary from one switch
//    type to another. If the value is too long, we risk locking
//    out genuine multiple key presses as well as bounces.
#define LOCKOUT_SCANS 100

typedef uint8_t BOOL;

// Number of columns in the key matrix. In this design, columns are
//   outputs, and the column pins will be set low in sequence to
//   do the scan
#define NUM_COLUMNS 4
//#define NUM_COLUMNS 11 // For future expansion

// Number of rows in the key matrix. In this design, rows are inputs,
//   and are read in sequence whilst each column pin is set low.
//   This means that a '0' indicates that a key is pressed, and a 
//   '1' indicates that the key is up
#define NUM_ROWS 4

// Define the Arduino pins that will be used as outputs to scan
//   the matrix columns. The first pin in this list is column zero.
uint8_t columns[NUM_COLUMNS] = 
  {5, 4, 3, 2};

// Define the Arduino pins that will be used as inputs, attached to 
//   the matrix rows. The first pin in this list is row zero. 
uint8_t rows[NUM_ROWS] = 
  {7, 16, 14, 15};

// A KeyState is an integer representing the state of the individual
//   key in the finite state machine. 
typedef uint8_t KeyState;

// The keysyms that will be emitted when the key at a specific
//   row and column is pressed. This matrix should look rather
//   like the keypad itself. 
static const uint8_t istickBtn[NUM_COLUMNS][NUM_ROWS] = 
  {
  {0, 1, 2, 3},
  {4, 5, 6, 7},
  {8, 9, 10, 11},
  {0, 1, 2, 3}
  };

static const uint8_t istickNbr[NUM_COLUMNS][NUM_ROWS] = 
  {
    {kAUTO_IX, kAUTO_IX, kAUTO_IX, kAUTO_IX},
    {kAUTO_IX, kAUTO_IX, kAUTO_IX, kAUTO_IX},
    {kAUTO_IX, kAUTO_IX, kAUTO_IX, kAUTO_IX},
    {kTELEOP_IX, kTELEOP_IX, kTELEOP_IX, kTELEOP_IX}
  };

// These are the states the key can be in (see state diagram above).
// All keys start in the wait_press state.
#define KEYSTATE_WAIT_PRESS 0
#define KEYSTATE_PRESS_LOCKOUT 1
#define KEYSTATE_WAIT_RELEASE 2
#define KEYSTATE_RELEASE_LOCKOUT 3

// The keystates matrix indicates the current state of each
//   key in the matrix.
static KeyState keystates[NUM_COLUMNS][NUM_ROWS];

// Lockout is the number of scan cycles that a key must wait, to
//   move out of the press_lockout or release_lockout states. 
// An entry for a specific key is set just after emitting a keypress
//   or key release event to the host. The values are all decremented
//   on each scan cycle, unless they are zero. The state transition
//   occurs when the lockout value goes from 1 to 0
static uint16_t lockout[NUM_COLUMNS][NUM_ROWS];


#define ANALOG_CNT 3

const static uint8_t axisIn[ANALOG_CNT] =
{
  A2,
  A3,
  A10 
};

const static joystickAxis axisMap[ANALOG_CNT] =
{
  X_AXIS,
  Y_AXIS,
  Z_AXIS
};

const static uint8_t axisStick[ANALOG_CNT] =
{
  kTELEOP_IX,
  kTELEOP_IX,
  kTELEOP_IX
};

#define ROTARY_SW_CNT 3

const static uint8_t rotSwIn[ROTARY_SW_CNT] =
{
  A0,
  A1,
  A7
};

const static uint8_t rotSwContacts[ROTARY_SW_CNT] =
{
  6,
  6,
  6
};

static uint16_t rotSwIncrement[ROTARY_SW_CNT] = 
{
  0,
  0,
  0
};

static uint16_t rotSwMidpoint[ROTARY_SW_CNT] = 
{
  0,
  0,
  0
};

const static uint8_t rotSwMap[ROTARY_SW_CNT] =
{
  0,
  1,
  0
};

const static uint8_t rotSwStick[ROTARY_SW_CNT] =
{
  kAUTO_IX,
  kAUTO_IX,
  kTELEOP_IX
};


void setup() {

  for (int index = 0; index < ROTARY_SW_CNT; index++)
  {
    rotSwIncrement[index] = 1023 / (rotSwContacts[index] - 1);
    rotSwMidpoint[index] = rotSwIncrement[index] / 2;
  }

  for (int index = 0; index < JOYSTICK_COUNT; index++)
  {
    Joystick[index].begin(false);
  }
  for (uint8_t i = 0; i < NUM_COLUMNS; i++)
  {
    pinMode (columns[i], INPUT);
  }
  for (uint8_t i = 0; i < NUM_ROWS; i++)
  {
    // Use INPUT_PULLUP so we don't need external resistors
    pinMode (rows[i], INPUT_PULLUP);
  }
  // Clear the key states
  memset (keystates, KEYSTATE_WAIT_PRESS, sizeof (keystates));

  memset (lockout, 0, sizeof (lockout));

  Keyboard.begin();

}

/**
 * keytest_emit()
 * Emit the specified keysym. Indicate a keypress if pressed
 * is 1, and a key release if it is 0.
 */
static void keytest_emit (uint8_t button, uint8_t stickNbr, BOOL pressed) {
  if (stickNbr == kKEYBOARD_IX) {
    if (pressed) {
      Keyboard.press(' ');
    } else {
      Keyboard.release(' ');
    }
  } else { 
    // if the button number is out of bounds, ignore it
    if (kBUTTON_CNT[stickNbr] > button) {
      if (pressed) {
        Joystick[stickNbr].pressButton(button);
      }
      else {
        Joystick[stickNbr].releaseButton(button);
      }
    }
  }
}

/*
 * keytest_do_row_col()
 * On each scan, handle a specific row,column combination. On entry
 * pressed==1 indicates that the key is down, pressed==0 
 * indicates that it is up.
 */
static void keytest_do_row_col (uint8_t row, uint8_t col, BOOL pressed, BOOL timeout) {
  uint8_t oldstate = keystates[col][row];
  uint8_t newstate = oldstate;
  switch (oldstate) {
    case KEYSTATE_WAIT_PRESS:
      if (pressed) {
        lockout[col][row] = LOCKOUT_SCANS; 
        keytest_emit (istickBtn[col][row], istickNbr[col][row], true);
        newstate = KEYSTATE_PRESS_LOCKOUT;
      }
      break;

    case KEYSTATE_PRESS_LOCKOUT:
      if (timeout) {
        newstate = KEYSTATE_WAIT_RELEASE;
      }
      break;

    case KEYSTATE_WAIT_RELEASE:
      if (!pressed) {
        lockout[col][row] = LOCKOUT_SCANS; 
        keytest_emit (istickBtn[col][row], istickNbr[col][row], false); 
        newstate = KEYSTATE_RELEASE_LOCKOUT;
	    }
      break;

    case KEYSTATE_RELEASE_LOCKOUT:
      if (timeout) {
        newstate = KEYSTATE_WAIT_PRESS;
      }
      break;
  }

  keystates[col][row] = newstate;
}

/*
 check axis assignment based on Joystick_ docs
 axisIn is the analog port
 axisMap is the enum mapping to a joystick analog value
 axisStick is the joystick entry
*/

static void assignPOV (uint8_t rotSwIx) {
  uint16_t rotSwVal = analogRead(rotSwIn[rotSwIx]);
  uint16_t POVSwHeading = (((rotSwVal * (rotSwContacts[rotSwIx] - 1)) + rotSwMidpoint[rotSwIx]) / 1023) * 45;
  // add test for joystick pov count
  if (kPOV_CNT[rotSwStick[rotSwIx]] > rotSwMap[rotSwIx]) {
    Joystick[rotSwStick[rotSwIx]].setHatSwitch(rotSwMap[rotSwIx], POVSwHeading);
  }
}

static void assignAxis (uint8_t analogIn) {
  int axisValue = analogRead(axisIn[analogIn]);
  switch (axisMap[analogIn]) {
    case X_AXIS:
      if (kIS_X_AXIS[axisStick[analogIn]])
       Joystick[axisStick[analogIn]].setXAxis(axisValue);
      break;
    case Y_AXIS:
      if (kIS_Y_AXIS[axisStick[analogIn]])
      Joystick[axisStick[analogIn]].setYAxis(axisValue);
      break;
    case Z_AXIS:
      if (kIS_Z_AXIS[axisStick[analogIn]])
        Joystick[axisStick[analogIn]].setZAxis(axisValue);
      break;
    case RX_AXIS:
      if (kIS_X_ROTATION[axisStick[analogIn]])
        Joystick[axisStick[analogIn]].setRxAxis(axisValue);    
      break;
    case RY_AXIS:
      if (kIS_Y_ROTATION[axisStick[analogIn]])
        Joystick[axisStick[analogIn]].setRyAxis(axisValue);
      break; 
    case RZ_AXIS:
      if (kIS_Z_ROTATION[axisStick[analogIn]])
        Joystick[axisStick[analogIn]].setRzAxis(axisValue);
      break;
    case RUDDER:
      if (kIS_RUDDER[axisStick[analogIn]])
        Joystick[axisStick[analogIn]].setRudder(axisValue);
      break;
    case THROTTLE:
      if (kIS_THROTTLE[axisStick[analogIn]])
        Joystick[axisStick[analogIn]].setThrottle(axisValue);
      break;
    case ACCELERATOR:
      if (kIS_ACCELRATOR[axisStick[analogIn]])
        Joystick[axisStick[analogIn]].setAccelerator(axisValue);
      break;
    case BRAKE:
      if (kIS_BRAKE[axisStick[analogIn]])
        Joystick[axisStick[analogIn]].setBrake(axisValue);
      break;
    case STEERING:
      if (kIS_STEERING[axisStick[analogIn]])
        Joystick[axisStick[analogIn]].setSteering(axisValue);
      break;
    default:
      break;
  }
}

/*
 * loop()
 * Scan all columns and rows once. Call keytest_do_row_col() for
 * each row/column pair.
 */
void loop() {

  // assignPOV
  for (uint8_t rotIx = 0; rotIx < ROTARY_SW_CNT; rotIx++) {
    assignPOV(rotIx);
  }
  
  // assignAxis
  for (uint8_t axisIx = 0; axisIx < ANALOG_CNT; axisIx++) {
    assignAxis(axisIx);
  }
  
  // For each row...
  for (uint8_t row = 0; row < NUM_ROWS; row++) {
    // ...set the row output low...
    pinMode (rows[row], OUTPUT);
    digitalWrite (rows[row], LOW);
    // ... and then read the state of each col.
    for (uint8_t col = 0; col < NUM_COLUMNS; col++) {
      uint16_t l = lockout[col][row];
      pinMode(columns[col], INPUT_PULLUP);
      BOOL v = digitalRead (columns[col]);
      pinMode(columns[col], INPUT);
      keytest_do_row_col (row, col, !v, l == 1);
      if (l > 0) {
        l--;
        lockout [col][row] = l;
      }
    }
    //delay(100); // We might need a delay when debugging
    pinMode (rows[row], INPUT);
  }

  for (int index = 0; index < JOYSTICK_COUNT; index++)
  {
    Joystick[index].sendState();
  }
}
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <AccelStepper.h>

const int INT_MAX = 32767;
const int NONE = -1;
const int TICK = 150; // microseconds

const int BUTTON_UP = LOW;
const int BUTTON_DOWN = HIGH;

// Run modes
const int MODE_DISABLED         = 0;
const int MODE_AUTO             = 1;
const int MODE_MANUAL           = 2;
const int MODE_SET_CLOSE_THRESH = 3;
const int MODE_SET_OPEN_THRESH  = 4;
const int MODE_MIN              = MODE_DISABLED;
const int MODE_MAX              = MODE_SET_OPEN_THRESH;

// Door positions
const int DOOR_POSITION_UNKNOWN = 0;
const int DOOR_POSITION_UP      = 1;
const int DOOR_POSITION_DOWN    = 2;
const int DOOR_POSITION_MAX     = DOOR_POSITION_DOWN;
const int DOOR_POSITION_MIN     = DOOR_POSITION_UNKNOWN;

// Door motion directions
const int DOOR_STATIONARY       = 0;
const int DOOR_MOVING_UP        = 1;
const int DOOR_MOVING_DOWN      = 2;

// EEPROM addresses
const int DOOR_POSITION_ADDRESS     = 0;
const int CURRENT_MODE_ADDRESS      = 1;
const int LIGHT_THRESH_DOWN_ADDRESS = 3;
const int LIGHT_THRESH_UP_ADDRESS   = 4;
const int EEPROM_UNSET              = 255;

// Motor constants
const int MOTOR_RPMS            = 60;

// Number of ticks beyond threshold to wait before changing door position
const int LIGHT_THRESH_TICKS_TO_CHANGE = 100;

// Motor constants
const int STEPS_PER_REVOLUTION = 200;

// Door-specific constants
// Number of revolutions to fully change door position
const int REVOLUTIONS_TO_CHANGE   = 100;

// Pins
// Arduino Uno / Buttons
const int PIN_MODE_BUTTON = 8;
const int PIN_UP_BUTTON   = 10;
const int PIN_DOWN_BUTTON = 9;
// Arduino Uno / LCD screen
const int PIN_LCD_RS      = 12;
const int PIN_LCD_ENABLE  = 11;
const int PIN_LCD_D4      = 5;
const int PIN_LCD_D5      = 4;
const int PIN_LCD_D6      = 3;
const int PIN_LCD_D7      = 2;
// Arduino Uno / Photocell
const int PIN_PHOTOCELL   = A0;
// Arduino Uno / Motor controller
const int PIN_MOTOR_1     = A1; // pin 2 on L293D
const int PIN_MOTOR_2     = A2; // pin 7 on L293D
const int PIN_MOTOR_3     = A3; // pin 15 on L293D
const int PIN_MOTOR_4     = A4; // pin 10 on L293D

// Global State variables
int CurrentMode           = MODE_AUTO;
int DoorPosition          = DOOR_POSITION_UNKNOWN; 
int DoorDirection         = DOOR_STATIONARY;
// Used for detecting button presses
int UpButtonState         = BUTTON_UP;
int UpButtonLastState     = BUTTON_UP;
int DownButtonState       = BUTTON_UP;
int DownButtonLastState   = BUTTON_UP;
int ModeButtonState       = BUTTON_UP;
int ModeButtonLastState   = BUTTON_UP;
// Used for light sensor
int LightCurrent          = 0;             // Current reading
int LightLast             = LightCurrent;  // Last reading
int LightThreshUp         = 100;
int LightThreshDown       = 10;
int LightThreshTicks      = 0;              // Ticks needed to start door position change

// Initialize the LCD 
LiquidCrystal Lcd(PIN_LCD_RS, PIN_LCD_ENABLE, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

// Initialize the stepper motor controller
AccelStepper Motor(AccelStepper::FULL4WIRE, PIN_MOTOR_1, PIN_MOTOR_2, PIN_MOTOR_3, PIN_MOTOR_4, true);

void setup() {
  
  // initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Set up pins
  pinMode(PIN_MODE_BUTTON,  INPUT);
  pinMode(PIN_UP_BUTTON,    INPUT);
  pinMode(PIN_DOWN_BUTTON,  INPUT);
  pinMode(PIN_PHOTOCELL,    INPUT);
  pinMode(PIN_MOTOR_1,      OUTPUT);
  pinMode(PIN_MOTOR_2,      OUTPUT);
  pinMode(PIN_MOTOR_3,      OUTPUT);
  pinMode(PIN_MOTOR_4,      OUTPUT);
  
  // set up the LCD's number of columns and rows:
  Lcd.begin(16, 2);
  // Print a message to the LCD.
  Lcd.print("eCoop v2.0");

  // Set up the motor
  Motor.setSpeed( ( MOTOR_RPMS / 60 ) * STEPS_PER_REVOLUTION );

  // Read door position from eeprom
  DoorPosition = EEPROM.read(DOOR_POSITION_ADDRESS);
  // Detect bad eeprom values and fix
  if ( DoorPosition > DOOR_POSITION_MAX || DoorPosition < DOOR_POSITION_MIN ) {
    DoorPosition = DOOR_POSITION_UNKNOWN;
    EEPROM.write(DOOR_POSITION_ADDRESS, DOOR_POSITION_UNKNOWN);
  }
  // Handle DoorPosition == DOOR_POSITION_UNKNOWN or never set
  if (DoorPosition == DOOR_POSITION_UNKNOWN || DoorPosition == EEPROM_UNSET ) {
    CurrentMode = MODE_MANUAL;
  } 

  // Read current mode from eeprom
  CurrentMode = EEPROM.read(CURRENT_MODE_ADDRESS);
  // Make sure value is in legal range. If not, fall back to AUTO mode
  if (CurrentMode > MODE_MAX || CurrentMode < MODE_MIN ) {
    CurrentMode = MODE_AUTO;
  }

  delay(1000);
  updateLCD();
}

void loop() {

  // Handle pressed buttons
  handleButton();

  handleLight();
  
  // Sleep to avoid busy wait
  delay(TICK);
}

void handleButton() {
  int button = pressedButton();
  if ( button == PIN_MODE_BUTTON ) {
    nextMode();
    updateLCD();
  } else if ( button == PIN_UP_BUTTON ) {
    handleButtonUp();
    updateLCD();
  } else if ( button == PIN_DOWN_BUTTON ) {
    handleButtonDown();
    updateLCD();
  }
}

// Returns code of button that has been pressed, if any
int pressedButton() {
  // Look at MODE button
  ModeButtonState = digitalRead(PIN_MODE_BUTTON);
  if ( ModeButtonState == BUTTON_DOWN && ModeButtonState != ModeButtonLastState ) {
    ModeButtonLastState = ModeButtonState;
    return PIN_MODE_BUTTON;
  }
  ModeButtonLastState = ModeButtonState;
  // Look at UP button
  UpButtonState = digitalRead(PIN_UP_BUTTON);
  if ( UpButtonState == BUTTON_DOWN && UpButtonState != UpButtonLastState ){
    UpButtonLastState = UpButtonState;
    return PIN_UP_BUTTON;
  }
  UpButtonLastState = UpButtonState;
  // Look at MODE pin
  DownButtonState = digitalRead(PIN_DOWN_BUTTON);
  if ( DownButtonState == BUTTON_DOWN && DownButtonState != DownButtonLastState ){
    DownButtonLastState = DownButtonState;
    return PIN_DOWN_BUTTON;
  }
  DownButtonLastState = DownButtonState;
  return NONE;
}

void nextMode() {
  CurrentMode++;
  if ( CurrentMode > MODE_MAX ) {
    CurrentMode = MODE_MIN;
  }
  // Save mode to memory
  EEPROM.write(CURRENT_MODE_ADDRESS, CurrentMode);

  // Clean up after for AUTO mode
  if ( CurrentMode != MODE_AUTO )  {
    LightThreshTicks = 0;
  }

  // Clean up after MANUAL mode
  if ( CurrentMode != MODE_MANUAL )  {
    if ( doorIsMoving() ) {
      stopDoor();
    }
  }
}

void handleButtonUp() {
  if ( CurrentMode == MODE_MANUAL ) {
    if ( DoorDirection == DOOR_STATIONARY ) {
      // Move door UP since we are in manual mode
      moveDoorUp(INT_MAX);
    } else {
      // Set the door position to UP and stop moving
      DoorPosition = DOOR_POSITION_UP;
      stopDoor();
    }
  } else if ( CurrentMode == MODE_SET_OPEN_THRESH ) {
    LightThreshUp++;
    Serial.print("MODE_SET_OPEN_THRESH ");
    Serial.println(LightThreshUp);
  } else if ( CurrentMode == MODE_SET_CLOSE_THRESH ) {
    LightThreshDown++;
    Serial.print("MODE_SET_CLOSE_THRESH ");
    Serial.println(LightThreshDown);
  }
}

void handleButtonDown() {
  if ( CurrentMode == MODE_MANUAL ) {
    if ( DoorDirection == DOOR_STATIONARY ) {
      // Move door DOWN since we are in manual mode
      moveDoorDown(INT_MAX);
    } else {
      // Door was not stationary so set current position to DOWN and stop it
      DoorPosition = DOOR_POSITION_DOWN;
      stopDoor();
    }
  } else if ( CurrentMode == MODE_SET_OPEN_THRESH ) {
    LightThreshUp = LightThreshUp - 1;
    Serial.print("MODE_SET_OPEN_THRESH ");
    Serial.println(LightThreshUp);
  } else if ( CurrentMode == MODE_SET_CLOSE_THRESH ) {
    LightThreshDown = LightThreshDown - 1;
    Serial.print("MODE_SET_CLOSE_THRESH ");
    Serial.println(LightThreshDown);
  }
}

void handleLight() {
  
  updateLight();

  if ( LightLast != LightCurrent ) {
    updateLCD();
  }

  // Keep going only if we are in AUTO mode
  if ( CurrentMode != MODE_AUTO ) {
    return;
  }
  
  // Determine if door should move up or down
  if ( LightCurrent > LightThreshUp ) {

    // See if we could potentially change door position
    // If not, there's nothing else to do here
    if ( DoorPosition == DOOR_POSITION_UP ) {
      return;
    }
    
    LightThreshTicks++;
    updateLCD();
    if ( LightThreshTicks > LIGHT_THRESH_TICKS_TO_CHANGE ) {
      // Start moving up
      openDoor();
      // Done moving door so reset ticks
      LightThreshTicks = 0;
    }
  } else if ( LightCurrent < LightThreshDown ) {

    // See if we could potentially change door position
    // If not, there's nothing else to do here
    if ( DoorPosition == DOOR_POSITION_DOWN ) {
      return;
    }
    
    LightThreshTicks++;
    updateLCD();
    if ( LightThreshTicks > LIGHT_THRESH_TICKS_TO_CHANGE ) {
      // Start moving down
      closeDoor();
      // Done moving door so reset ticks
      LightThreshTicks = 0;
    }
  } else {
    LightThreshTicks = 0;
  }
}

// Read light sensor and set global var
void updateLight() {
  LightLast = LightCurrent;
  LightCurrent = analogRead(PIN_PHOTOCELL);
}

void openDoor() {
  // If door is already up, just return
  if ( DoorPosition == DOOR_POSITION_UP ) {
    return;
  }
  moveDoorUp(REVOLUTIONS_TO_CHANGE);
}

void closeDoor() {
  // If door is already down, just return
  if ( DoorPosition == DOOR_POSITION_DOWN ) {
    return;
  }
  moveDoorDown(REVOLUTIONS_TO_CHANGE);
}

void moveDoorUp(int revolutions) {
  updateDoorStatus(DOOR_MOVING_UP, DOOR_POSITION_UNKNOWN);
  moveDoor(revolutions);
  updateDoorStatus(DOOR_STATIONARY, DOOR_POSITION_UP);
}

// Start moving door down
void moveDoorDown(int revolutions) {
  updateDoorStatus(DOOR_MOVING_DOWN, DOOR_POSITION_UNKNOWN);
  moveDoor(-revolutions);
  updateDoorStatus(DOOR_STATIONARY, DOOR_POSITION_DOWN);
}

// Moves door either up or down. 
// If revolutions is positive, door goes up; if negative, door goes down.
void moveDoor(int revolutions) {
  // Set the motor target position i.e. number of steps to move
  Motor.move(revolutions * (long) STEPS_PER_REVOLUTION);
  
  // Start stepping motor
  // for (int rev = 1; rev <= revolutions; rev++) {
  while (true) {

    // Handle buttons to allow abort
    handleButton();

    // Eject from loop if we stopped the door manually
    if ( DoorDirection == DOOR_STATIONARY ) {
      // Reset the stepper library
      break;
    }

    // Step motor and check if we should loop again
    bool stillRunning = Motor.run();
    if ( ! stillRunning )  {
      break;
    }
  }
}

// Stop door motion
void stopDoor() {
  // Update status
  DoorDirection = DOOR_STATIONARY;
  EEPROM.write(DOOR_POSITION_ADDRESS, DoorPosition);
}

bool doorIsMoving() {
  return ( DoorDirection != DOOR_STATIONARY );
}

void updateDoorStatus(int doorMoving, int doorPosition) {
  DoorDirection = doorMoving;
  DoorPosition = doorPosition;
  EEPROM.write(DOOR_POSITION_ADDRESS, doorPosition);
  updateLCD();
}

// Update LCD with current status
void updateLCD() {
  
  Lcd.clear();

  if ( CurrentMode <= MODE_MANUAL && CurrentMode >= MODE_DISABLED ) {
    // Render run mode 
    
    // Print first line
    
    Lcd.print("Mode: ");
    Lcd.print(prettyMode());
  
    Lcd.print(" Door: ");
    Lcd.print(prettyDoorPosition());

    // Render second line
    Lcd.setCursor(0, 1);
    Lcd.print(LightThreshDown);
    Lcd.setCursor(5, 1);
    Lcd.print(LightCurrent);
    Lcd.setCursor(10, 1);
    Lcd.print(LightThreshUp);
    Lcd.setCursor(15, 1);
    char doorDirection = prettyDoorDirection();
    int ticksToChange = prettyTicksToChange();
    if (doorDirection == ' ' && ticksToChange > 0 && ticksToChange < 10 ) {
      Lcd.print(prettyTicksToChange());
    } else {
      Lcd.print(doorDirection);
    }
  
  } else {
    // Render settings

    // Render first line
    Lcd.print("Set: ");
    if ( CurrentMode == MODE_SET_OPEN_THRESH ) {
      Lcd.print("Open Threshold");
    } else if ( CurrentMode == MODE_SET_CLOSE_THRESH ) {
      Lcd.print("Close Threshold");
    }

    // Render second line
    Lcd.setCursor(0, 1);
    if ( CurrentMode == MODE_SET_OPEN_THRESH ) {
      Lcd.print(LightThreshUp);
    } else if ( CurrentMode == MODE_SET_CLOSE_THRESH ) {
      Lcd.print(LightThreshDown);
    }
  }
}

char prettyMode() {
  if ( CurrentMode == MODE_DISABLED ) {
    return '0';
  } else if ( CurrentMode == MODE_AUTO ) {
    return 'A';
  } else if ( CurrentMode == MODE_MANUAL ) {
    return 'M';
  }
  return ' ';
}

char prettyDoorDirection() {
  if ( DoorDirection == DOOR_MOVING_UP ) {
    return 'U';
  } else if ( DoorDirection == DOOR_MOVING_DOWN ) {
    return 'D';
  }
  return ' ';
}

char prettyDoorPosition() {
  if ( DoorPosition == DOOR_POSITION_UNKNOWN ) {
    return '?';
  } else if ( DoorPosition == DOOR_POSITION_UP ) {
    return 'U';
  } else if ( DoorPosition == DOOR_POSITION_DOWN ) {
    return 'D';
  }
  return ' ';
}

int prettyTicksToChange() {
  return ( LightThreshTicks / (float) LIGHT_THRESH_TICKS_TO_CHANGE ) * 10;
}

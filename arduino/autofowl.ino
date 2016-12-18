#define DEBUG 1

// --------------------------------------
// -- Configurable Constants
// --------------------------------------

//-- Time --
// A tick is a single unit of time.
const int TICK_MS = 1000;
// Number of ticks to wait before changing photo state
const int TICKS_TO_CHANGE_STATE = 10;

//-- Door Stepper --
// Steps per rotation
// http://arduino-info.wikispaces.com/SmallSteppers
const long STEPPER_STEPS_PER_MOTOR_REV = 200;
// Stepper gear reduction
const long STEPPER_REDUCTION = 1;
// Length of time to pulse motor. Shorter pulse means faster rpm.
// in microseconds
const int STEPPER_PULSE_WIDTH = 1500;
// Number of revolutions needed to change door position
// Should be empirically determined since it depends on lenght of thread rod.
const int REV_TO_CHANGE_POSITION = 170;

//-- Sensor smoothing
// Apply sensor smoothing?
const bool SMOOTH_SENSORS = true;
// Number of ticks to smooth sensor values over
// Higher value makes values change slower
const int TICKS_TO_SMOOTH = 5;

// --------------------------------------
// -- Constants
// --------------------------------------

//-- Sensor and actuator pins
const int PHOTOCELL_PIN = A0;
const int LED_PIN = 13;
// Buttons
const int RUNMODE_BUTTON_PIN = 2;
const int UP_BUTTON_PIN = 4;
const int DOWN_BUTTON_PIN = 3;
// Stepper motor pins
const int STEPPER_PIN_1 = 8;
const int STEPPER_PIN_2 = 9;
const int STEPPER_PIN_3 = 10;
const int STEPPER_PIN_4 = 11;
// Potentiometers
const int UPPER_POT_PIN = A1;
const int LOWER_POT_PIN = A2;

// Ambient light states (aka day or night)
const int PHOTO_STATE_UNKNOWN = 0;
const int PHOTO_STATE_DARK = -1;
const int PHOTO_STATE_LIGHT = 1;

// -- Run modes
const int RUNMODE_PAUSE = 0;
// Automatically open or close the door
const int RUNMODE_AUTO = 1;
// Manually open/close door
const int RUNMODE_MANUAL = 2;
// for convenience
const int RUNMODE_MAX = RUNMODE_MANUAL;

// -- Door positions
const int DOOR_UNKNOWN = 0;
const int DOOR_CLOSED = -1;
const int DOOR_OPEN = 1;

//-- Door Stepper
// Steps per revolution
const long STEPPER_STEPS_PER_REV = STEPPER_STEPS_PER_MOTOR_REV * STEPPER_REDUCTION;
// number or stepper motor steps needed to go from one 
// position to another
long STEPS_TO_CHANGE_POSITION = REV_TO_CHANGE_POSITION * STEPPER_STEPS_PER_REV;

// --------------------------------------
// -- Runtime variables
// --------------------------------------

int photocell_value = 0;
// We start up in paused mode
int runmode = RUNMODE_PAUSE;
int last_runmode = RUNMODE_PAUSE;

// TODO rename
// Points at which we cross into day or night
// These will be changed be the potentiometers
int photo_dark_threshold = 100;
int photo_light_threshold = 800;

// Position of door
int current_position = DOOR_UNKNOWN;
int next_position = DOOR_UNKNOWN;

// Night or day.
int current_photo_state = PHOTO_STATE_UNKNOWN;
int next_photo_state = PHOTO_STATE_UNKNOWN;
// Number of ticks passed in current photo state
int ticks_in_next_photo_state = 0;

// --------------------------------------
// -- Functions
// --------------------------------------

void debugToSerial() {
  // print the results to the serial monitor:
  #if DEBUG
  Serial.print("runmode=" );
  Serial.print(runmode);
  Serial.print(",position=" );
  Serial.print(current_position);
  Serial.print("->" );
  Serial.print(next_position);
  Serial.print(",photo_state=" );
  Serial.print(current_photo_state);
  Serial.print("->" );
  Serial.print(next_photo_state);
  Serial.print(",ticks_in_next_photo_state=" );
  Serial.print(ticks_in_next_photo_state);
  Serial.print(",photocell=" );
  Serial.print(photocell_value);      
  Serial.print(" thresholds=" );
  Serial.print(photo_dark_threshold);      
  Serial.print("," );
  Serial.print(photo_light_threshold);
  Serial.print("\r\n");
  #endif
}

int smooth(int new_value, int smoothed_value) {
  return smoothed_value + ((new_value - smoothed_value) / TICKS_TO_SMOOTH);
}

//-- BEGIN stepper motor functions

void motor_step_1(){
  digitalWrite(STEPPER_PIN_1, LOW);   
  digitalWrite(STEPPER_PIN_2, HIGH);   
  digitalWrite(STEPPER_PIN_3, HIGH);   
  digitalWrite(STEPPER_PIN_4, LOW);   
  delayMicroseconds(STEPPER_PULSE_WIDTH);
}
void motor_step_2(){
  digitalWrite(STEPPER_PIN_1, LOW);   
  digitalWrite(STEPPER_PIN_2, HIGH);   
  digitalWrite(STEPPER_PIN_3, LOW);   
  digitalWrite(STEPPER_PIN_4, HIGH);   
  delayMicroseconds(STEPPER_PULSE_WIDTH);
}
void motor_step_3(){
  digitalWrite(STEPPER_PIN_1, HIGH);   
  digitalWrite(STEPPER_PIN_2, LOW);   
  digitalWrite(STEPPER_PIN_3, LOW);   
  digitalWrite(STEPPER_PIN_4, HIGH);   
  delayMicroseconds(STEPPER_PULSE_WIDTH);
}
void motor_step_4(){
  digitalWrite(STEPPER_PIN_1, HIGH);   
  digitalWrite(STEPPER_PIN_2, LOW);   
  digitalWrite(STEPPER_PIN_3, HIGH);   
  digitalWrite(STEPPER_PIN_4, LOW);   
  delayMicroseconds(STEPPER_PULSE_WIDTH);
}
void motor_off(){
  digitalWrite(STEPPER_PIN_1, LOW);   
  digitalWrite(STEPPER_PIN_2, LOW);   
  digitalWrite(STEPPER_PIN_3, LOW);   
  digitalWrite(STEPPER_PIN_4, LOW);   
}

void move_stepper_clockwise() {
  // Move stepper clockwise 4 steps
  motor_step_1(); 
  motor_step_2();
  motor_step_3();
  motor_step_4();
}

void move_stepper_counterclockwise() {
  // Move stepper counterclockwise 4 steps
  motor_step_4();
  motor_step_3();
  motor_step_2();
  motor_step_1(); 
}

//-- END Stepper motor functions

void closeDoor() {
  // Close the door
  
  #if DEBUG
  Serial.print("Closing door.\r\n");
  #endif

  // Wait 2 seconds for buttons to be released. Blink for feedback.
  digitalWrite(LED_PIN, LOW);
  delay(2000);
  digitalWrite(LED_PIN, HIGH);

  // Set current position unknown since it will be in the middle
  // for a while
  current_position = DOOR_UNKNOWN;
  
  // Blocks until move finished
  // Divide by 4 since we always go 4 steps
  bool broke_out = false;
  for (int i=0; i<=STEPS_TO_CHANGE_POSITION/4; i++){
    
    // Abort if runmode button pressed
    if ( digitalRead(RUNMODE_BUTTON_PIN) == HIGH ) {
      current_position = DOOR_UNKNOWN;
      broke_out = true;
      break;
    } else if ( digitalRead(UP_BUTTON_PIN) == HIGH ) {
      current_position = DOOR_OPEN;
      broke_out = true;
      break;
    } else if ( digitalRead(DOWN_BUTTON_PIN) == HIGH ) {
      current_position = DOOR_CLOSED;
      broke_out = true;
      break;
    }    
    move_stepper_clockwise();  
  }
  motor_off();
  
  if (!broke_out) {
    current_position = DOOR_CLOSED;
  }
}

void openDoor() {
  // Open the door

  #if DEBUG
  Serial.print("Opening door.\r\n");
  #endif

  // Wait 2 seconds for buttons to be released. Blink for feedback.
  digitalWrite(LED_PIN, LOW);
  delay(2000);
  digitalWrite(LED_PIN, HIGH);

  // Set current position unknown since it will be in the middle
  // for a while
  current_position = DOOR_UNKNOWN;

  // Blocks until move finished
  // Divide by 4 since we always go 4 steps
  bool broke_out = false;
  for (int i=0; i<=STEPS_TO_CHANGE_POSITION/4; i++){
    
    // Abort if runmode button pressed
    if ( digitalRead(RUNMODE_BUTTON_PIN) == HIGH ) {
      current_position = DOOR_UNKNOWN;
      broke_out = true;
      break;
    } else if ( digitalRead(UP_BUTTON_PIN) == HIGH ) {
      current_position = DOOR_OPEN;
      broke_out = true;
      break;
    } else if ( digitalRead(DOWN_BUTTON_PIN) == HIGH ) {
      current_position = DOOR_CLOSED;
      broke_out = true;
      break;
    }
    move_stepper_counterclockwise();  
  }
  motor_off();

  if (!broke_out) {
    current_position = DOOR_OPEN;
  }
}

void updateFromSensors() {
  // Update all of our sensor variables
  
  // read the analog in value:
  int new_photocell_value = analogRead(PHOTOCELL_PIN);

  // Possibly smooth out input over time
  if (SMOOTH_SENSORS) {
    // photocell_value = photocell_value + ((new_photocell_value - photocell_value) / TICKS_TO_CHANGE_STATE);
    photocell_value = smooth(new_photocell_value, photocell_value);
  } else {
    photocell_value = new_photocell_value;
  }
}

void updateBoundsFromPots() {
  int new_photo_light_threshold = analogRead(UPPER_POT_PIN);
  int new_photo_dark_threshold = analogRead(LOWER_POT_PIN);
  
  // On these pots, fully left is 1024, fully right is 0
  // so invert the values
  new_photo_light_threshold = abs(new_photo_light_threshold - 1024);
  new_photo_dark_threshold = abs(new_photo_dark_threshold - 1024);

  // Possibly smooth out input over time
  if (SMOOTH_SENSORS) {
    photo_light_threshold = smooth(new_photo_light_threshold, photo_light_threshold);
    photo_dark_threshold = smooth(new_photo_dark_threshold, photo_dark_threshold);
  } else {
    photo_light_threshold = photo_light_threshold;
    photo_dark_threshold = photo_dark_threshold;
  }
}

void calcPhotoState() {
  
  // TODO Need to handle circumstances where we get temporary light or dark
  // TODO Handle edge case where we jump from light to dark
  
  // Set photo state variables
  if ( photocell_value < photo_dark_threshold && 
       current_photo_state != PHOTO_STATE_DARK ) {
    // Crossed threshold into dark
    next_photo_state = PHOTO_STATE_DARK;
    // Increment tick counter
    ticks_in_next_photo_state += 1;
  } else if ( photocell_value > photo_light_threshold  && 
              current_photo_state != PHOTO_STATE_LIGHT ) {
    // Crossed threshold into light
    next_photo_state = PHOTO_STATE_LIGHT;
    // Increment tick counter
    ticks_in_next_photo_state += 1;    
  } else if ( photocell_value < photo_light_threshold && 
              photocell_value > photo_dark_threshold ) {
    // Photo cell is between light and dark, so time of day is unknown
    next_photo_state = PHOTO_STATE_UNKNOWN;
    // Restart counter
    ticks_in_next_photo_state = 0;
  } // else Photo state unchanged between two thresholds
  
  // Actually change the photo state if needed
  if ( ticks_in_next_photo_state >= TICKS_TO_CHANGE_STATE ) {
    // Changing photo state
    current_photo_state = next_photo_state;
    ticks_in_next_photo_state = 0;
  }
}

void doRunmodePause() {
  // Handle pausing
  // User feedback. led off
  digitalWrite(LED_PIN, LOW);
}

void doRunmodeAuto() {
  // Handle auto mode
  // Note: calcPhotoState() is called by main loop. Don't do it again.

  // Warn user then return immediately if door state is unknown
  if (current_position == DOOR_UNKNOWN) {
    // If current door position is unknown, we cant do anything
    // Warn user that they need to reset door
    for (int i=0; i<10; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(50);
    }
    return;
  }

  // User feedback. led fast blink
  digitalWrite(LED_PIN, HIGH);
  delay(10);
  digitalWrite(LED_PIN, LOW);

  // Open or close door
  if ( current_photo_state == PHOTO_STATE_DARK && current_position != DOOR_CLOSED ) {
    closeDoor();
  } else if ( current_photo_state == PHOTO_STATE_LIGHT && current_position != DOOR_OPEN) {
    openDoor();
  } // else do nothing
}

void doRunmodeManual() {
  // Handel manual open/close mode

  // User feedback. led on.
  digitalWrite(LED_PIN, HIGH);

  if ( digitalRead(UP_BUTTON_PIN) == HIGH ) {
    openDoor();
  } else if ( digitalRead(DOWN_BUTTON_PIN) == HIGH ) {
    closeDoor();
  }
  // else no button input
}

void setup() {
  #if DEBUG
  Serial.begin(9600); 
  #endif
  
  // Init led
  pinMode(LED_PIN, OUTPUT);
  
  // TODO Since we default to dark and closed, close door.
  // When we start, door position should be DOOR_UNKNOWN
  
  // Init stepper motor
  pinMode(STEPPER_PIN_1, OUTPUT);     
  pinMode(STEPPER_PIN_2, OUTPUT);     
  pinMode(STEPPER_PIN_3, OUTPUT);     
  pinMode(STEPPER_PIN_4, OUTPUT);   
}

void loop() {
    
  // Always keep our sensor readings up to date
  updateFromSensors();
  updateBoundsFromPots();
  calcPhotoState();
  
  if ( digitalRead(RUNMODE_BUTTON_PIN) == HIGH ) {
    // Step to next runmode
    runmode += 1;
    // Flip around to runmode 0 (what it is doesnt matter) 
    // if we exceed the highest runmode
    if ( runmode > RUNMODE_MAX ) {
      runmode = 0;
    }
    
    #if DEBUG
    Serial.print("Changed runmode to ");
    Serial.print(runmode);
    Serial.print("\r\n");
    #endif
  }

  // Reset photo state to avoid moving immediately
  // This will require a photo state change to make it do 
  // something automatically
  if (runmode != last_runmode) {  
    current_photo_state = 0;
    next_photo_state = 0;
  }

  if ( runmode == RUNMODE_PAUSE ) {
    doRunmodePause();
    debugToSerial();
    delay(TICK_MS);
  } else if ( runmode == RUNMODE_AUTO ) {
    doRunmodeAuto();    
    debugToSerial();
    // Wait N milliseconds
    delay(TICK_MS);
  } else if ( runmode == RUNMODE_MANUAL ) {
    doRunmodeManual();
    debugToSerial();    
    // Wait N milliseconds
    delay(TICK_MS);
  } else {
    // Unknown runmode. Just delay to avoid hammering cpu.
    // Note: This shouldnt be possible.
    delay(TICK_MS);
  }
  
  // Save what will soon become the previous runmode
  last_runmode = runmode;
}


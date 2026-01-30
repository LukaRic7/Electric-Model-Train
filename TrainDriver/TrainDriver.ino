/**
 * @file    TrainDriver.ino
 * @author  Luka Jacobsen
 * @brief   Bidirectional motor control system for a LEGO model train.
 * @date    2026-01-22
 * * @details This file handles wireless packet decoding, motor ramping, and an
 * autopilot state machine, aswell as manual control and headlights. Designed
 * for an H-Bridge driver.
 * * HARDWARECONNECTIONS:
 * - Pin 8:  Motor Forward    (Digital)
 * - Pin 7:  Motor Backward   (Digital)
 * - Pin 6   Motor Speed      (Digital, PWM)
 * - Pin 13: Headlights       (Digital)
 * - Pin 9:  Sensor Trigger   (Digital, PWM)
 * - Pin 10: Sensor Echo      (Digital, PWM)
 * - Pin 4:  Station Sensor   (Digital)
 */

// Include libraries to receive data
#include <RH_ASK.h>
#include <SPI.h>

// Create receiver driver
RH_ASK rf_driver;

// ========================================================================== //
// CONFIGURATION                                                              //
// ========================================================================== //

// -------- Speeds (0-255) --------- //
const uint8_t SPEED_CRUISE = 255;   // Full forwards
const uint8_t SPEED_SLOW   = 159;   // 25% power forwards
const uint8_t ACCEL_RATE   = 40;    // +40 pr second
const uint8_t DECEL_RATE   = 60;    // -60 pr second
const uint8_t DECEL_RATE_E = 120;   // -120 pr second when emergency stopped
const uint8_t DEADZONE     = 10;    // +- center

// --------- Timings (ms) ---------- //
const unsigned int STATION_DWELL_TIME = 5*1000; // (ms) 2 bytes, max 65 sec

// --------- Emergency (cm) --------- //
const uint8_t EMERGENCY_MARGIN_STOP   = 20;   // 40cm or closer, stop
const uint8_t EMERGENCY_MARGIN_RETURN = 30;   // 60cm away, cancel emergency

// ========================================================================== //
// PIN DEFINITIONS                                                            //
// ========================================================================== //

// --------- Motor Drivers --------- //
const uint8_t MOTOR_FORWARD  = 8;
const uint8_t MOTOR_BACKWARD = 7;
const uint8_t MOTOR_SPEED    = 6;

// ---------- Transmitter ---------- //
const uint8_t TRANSMIT_RECIEVE = 0;

// ---------- Headlights ----------- //
const uint8_t HEADLIGHT_PIN = 13;

// ------------ Sensors ------------ //
const uint8_t SENSOR_TRIGGER = 9;
const uint8_t SENSOR_ECHO    = 10;
const uint8_t STATION_SENSOR = 4;

// ========================================================================== //
// ENUMS / STRUCTS                                                            //
// ========================================================================== //

/**
 * @brief Operating modes of the train, used by the state machine.
 */
enum TrainState : uint8_t {
  STOPPED,          // Zero power, train stopped
  MANUAL_CONTROL,   // Direct control via remote
  AUTO_CRUISING,    // Autopilot constant speed
  AUTO_SLOWING,     // Autopilot slowing for station approach
  EMERGENCY_STOPPED // Something blocked the front sensor
};

/**
 * @brief Container for wireless control data. Sent from the remote to the
 * train every import change.
 */
struct Packet {
  uint8_t speed;     // Mapped pot speed (0-255)
  bool autopilot;    // True = Autopilot enabled
  bool headlights;   // True = LED Headlights enabled
};

// ========================================================================== //
// RUNTIME GLOBAL VARIABLES                                                   //
// ========================================================================== //

// --------- Train States ---------- //
TrainState train_state           = STOPPED;   // Current state of the train
bool autopilot_mode              = false;     // True = Autopilot enabled
bool headlights_on               = false;     // True = LED Headlights enabled
unsigned int station_dwell_timer = 0;         // (ms) Timer for station dwell
bool in_emergency_stop           = false;     // True = Emergency stopped
uint8_t station_trigger_count    = 0;         // Counts triggers (0, 1, 2)
bool last_station_sensor         = false;     // Edge detection

// ------------ Speeds ------------- //
uint8_t manual_speed = 0;   // Manual speed receieved from remote control
uint8_t target_speed = 0;   // Target speed train is moving towards
uint8_t train_speed  = 0;   // The actual train speed

// ========================================================================== //
// LIFECYCLE FUNCTIONS                                                        //
// ========================================================================== //

/**
 * @brief Called once on startup by system.
 */
void setup() {
  // Start serial (UART) on baudrate 9600
  Serial.begin(9600);

  // Initialize ASK driver
  rf_driver.init();

  // Set motor pin modes
  pinMode(MOTOR_FORWARD, OUTPUT);
  pinMode(MOTOR_BACKWARD, OUTPUT);
  pinMode(MOTOR_SPEED, OUTPUT);

  // Set headlights pin mode
  pinMode(HEADLIGHT_PIN, OUTPUT);

  // Set transmitter recieve pin
  pinMode(TRANSMIT_RECIEVE, INPUT);

  // Set the sensor pins
  pinMode(SENSOR_TRIGGER, OUTPUT);
  pinMode(SENSOR_ECHO, INPUT);
  pinMode(STATION_SENSOR, INPUT_PULLUP);

  // Set the initial motor speed to 0
  setMotor(0);
}

/**
 * @brief Called every time the processor can.
 */
void loop() {
  // Call main functions, keep loop() clean
  checkSensors();
  readInputs();
  updateStateMachine();
  applyOutputs();

  // !!! DEBUG !!! ----- OUTPUT LOG ----- !!! DEBUG !!! //
  Serial.println("Target Speed: " + String(target_speed)
    + " | Train Speed: " + String(train_speed)
    + " | Driving: " + (train_speed == 0 ? "Stopped" :
      (train_speed > 127 ? "Forwards" : "Backwards"))
    + " | Autopilot: " + (autopilot_mode ? "ON" : "OFF")
    + " | Headlights: " + (headlights_on ? "ON" : "OFF")
    + " | Emergency: " + (in_emergency_stop ? "ON" : "OFF")
    + " | Station: " + (last_station_sensor ? "ON" : "OFF") + String(millis() - station_dwell_timer)
    + " | State: " + String(train_state));
    // !!! DEBUG !!! -- ^^ -- ^^ -- ^^ -- !!! DEBUG !!! //
  /*
  */
}

// ========================================================================== //
// Main Functions                                                             //
// ========================================================================== //

/**
 * @brief Check if the train should perform an emergency stop
 */
void checkSensors() {
  // Clear trigger
  digitalWrite(SENSOR_TRIGGER, LOW);
  delay(2);
  
  // Fire a pulse for 10 ms
  digitalWrite(SENSOR_TRIGGER, HIGH);
  delay(10);
  digitalWrite(SENSOR_TRIGGER, LOW);

  // Read the echo, and calculate the distance
  unsigned long duration = pulseIn(SENSOR_ECHO, HIGH);
  unsigned int distance  = duration * 0.034 / 2; // Speed of sound, travel 2 way

  // Check if emergency flag should raise or lower 
  if (distance < EMERGENCY_MARGIN_STOP)
    in_emergency_stop = true;
  else if (in_emergency_stop && distance > EMERGENCY_MARGIN_RETURN)
    in_emergency_stop = false;

  // Read station sensor
  bool current_station = digitalRead(STATION_SENSOR);

  // Rise edge detection
  if (current_station && !last_station_sensor)
    station_trigger_count++;

  last_station_sensor = current_station;
}

/**
 * @brief Read the inputs from the serial and load their values into the global
 * variables.
 */
void readInputs() {
  uint8_t buf[15];
  uint8_t buflen = sizeof(buf);
  if (rf_driver.recv(buf, &buflen)) {
    //Serial.print(buf[0], BIN);
    //Serial.println(buf[1], BIN);

    Packet pkt;
    decodeStreamByte(buf[0], pkt);
    if (decodeStreamByte(buf[1], pkt)) {
      // Packet is fully received, update variables
      manual_speed   = pkt.speed;
      autopilot_mode = pkt.autopilot;
      headlights_on  = pkt.headlights;
      Serial.println(headlights_on);
    }
  }
}

/**
 * @brief State machine for the train, uses switch control flow state.
 */
void updateStateMachine() {
  switch (train_state) {
    // Train is emergency stopped
    case EMERGENCY_STOPPED:
      if (!in_emergency_stop) {
        station_trigger_count = 0;
        train_state = autopilot_mode ? AUTO_CRUISING : MANUAL_CONTROL;
      }
      break;

    // Train is stopped
    case STOPPED:
      if (in_emergency_stop)
        train_state = EMERGENCY_STOPPED;
      else if (millis() - station_dwell_timer >= STATION_DWELL_TIME) {
        station_trigger_count = 0;
        train_state = AUTO_CRUISING;
      }
      break;

    // Train is in manual drive mode
    case MANUAL_CONTROL:
      if (in_emergency_stop)
        train_state = EMERGENCY_STOPPED;
      else if (autopilot_mode) {
        station_trigger_count = 0;
        train_state = AUTO_CRUISING;
      }
      break;

    // Trains autopilot is cruising
    case AUTO_CRUISING:
      if (in_emergency_stop)
        train_state = EMERGENCY_STOPPED;
      else if (!autopilot_mode)
        train_state = MANUAL_CONTROL;
      else if (station_trigger_count == 1)
        train_state = AUTO_SLOWING;
      break;
    
    // Train is approaching station
    case AUTO_SLOWING:
      if (in_emergency_stop)
        train_state = EMERGENCY_STOPPED;
      else if (station_trigger_count >= 2) {
        station_dwell_timer = millis();
        train_state = STOPPED;
      }
      break;
  }
}

/**
 * @brief Writes to outputs, specifically, sets the train target speed, and
 * updates the headlights state.
 */
void applyOutputs() {
  // Set headlight state
  digitalWrite(HEADLIGHT_PIN, headlights_on ? HIGH : LOW);

  switch (train_state) {
    // Train needs to be stopped
    case EMERGENCY_STOPPED:
    case STOPPED:
      target_speed = 127;
      break;
    
    // Train is in manual control
    case MANUAL_CONTROL:
      target_speed = manual_speed;
      break;

    // Trains autopilot is cruising
    case AUTO_CRUISING:
      target_speed = SPEED_CRUISE;
      break;

    // Trains autopilot is slowing down
    case AUTO_SLOWING:
      target_speed = SPEED_SLOW;
      break;
  }

  // Set the motor state using a linear ramp up
  setMotor(linearSpeedRamp(target_speed));
}

// ========================================================================== //
// Helper Functions                                                           //
// ========================================================================== //

/**
 * @brief Decodes the first and second byte from the serial stream, and puts
 * them into a packet, passed by reference.
 * @param byte_in The byte from the serial read.
 * @param out The reference to the packet currently being built.
 * @return bool, true if the packet is done being built, false if not.
 */
bool decodeStreamByte(uint8_t byte_in, Packet &out) {
  // Static variables for the next call
  static uint8_t last_check      = 0;        // Store an ID, consistancy check
  static uint8_t speed_h         = 0;        // Hold the top 5 bits of the speed
  static bool waiting_for_second = false;    // Flag, tells if we are mid-packet

  uint8_t check = byte_in >> 6;              // Shift to right, isolate check
  bool frame    = (byte_in >> 5) & 0x01;     // Shift to right, mask the frame

  // First byte - the setup
  if (frame == 0) {
    speed_h            = byte_in & 0x1F;     // Mask the last 5 bits
    last_check         = check;              // Remember the check ID
    waiting_for_second = true;               // Set the flag
    return false;                            // Packet is not complete yet
  }

  // Second byte - but never saw the first
  if (!waiting_for_second)
    return false;                            // Out of sync, discard

  // Check doesn't match
  if (check != last_check) {
    waiting_for_second = false;              // Lower flag
    return false;                            // Check mismatch
  }

  // Reassemble packet
  uint8_t speed_l = (byte_in >> 2) & 0x07;   // Shift to the right, then mask

  out.speed      = (speed_h << 3) | speed_l; // Shift prev to left, then append
  out.autopilot  = (byte_in >> 1) & 0x01;    // Shift one to right, then mask
  out.headlights = byte_in & 0x01;           // Mask the last bit

  // Packet is valid
  waiting_for_second = false;
  return true;
}

/**
 * @brief Uses a linear ramp to gratually increase the speed.
 * @param target The target speed 0-255 int.
 * @return int, the speed at which to move the train.
 */
uint8_t linearSpeedRamp(int target) {
  // Static variables to remember for the next call
  static float current           = 0;   // The current speed
  static unsigned long last_time = 0;   // Store when the previous call was made

  unsigned long now = millis();         // Get timestamp
  float dt = (now -last_time) / 1000.0; // Get delta time in seconds
  if (dt <= 0) dt = 0.001;              // Check if enough time passed (1ms)
  last_time = now;                      // Update the last time called variable

  if (current < target) {
    // Target is bigger, accellerate
    current += ACCEL_RATE * dt;
    // Check if current exceded target
    if (current > target)
      current = target;
  } else if (current > target) {
    // Apply emergency break decel rate if needed
    if (in_emergency_stop)
      current -= DECEL_RATE_E * dt;
    else
      // Target is less, decellerate
      current -= DECEL_RATE * dt;
    
    // Check if current is below target
    if (current < target)
      current = target;
  }

  // Return the clamped current value to 1 byte
  return constrain((uint8_t)current, 0, 255);
}

/**
 * @brief Set the motor speed and direction based on 1 byte input.
 * @param speed The speed, non negative, artifical zero on center.
 */
void setMotor(uint8_t speed) {
  // Define scope wide variables
  uint8_t pwm   = 0;       // Pulse-Width-Modulation, actual speed to set
  bool forward  = false;   // Flag for driving forwards
  bool backward = false;   // Flag for driving backwards

  // Define thresholds (deadzones) around the center (127)
  int upper_threshold = 127 + DEADZONE;
  int lower_threshold = 127 - DEADZONE;

  // Move forwards
  if (speed > upper_threshold) {
    // Map the speed from threshold  
    pwm = (uint8_t)map(speed, upper_threshold, 255, 0, 255);
    forward = true;
  // Move backwards
  } else if (speed < lower_threshold) {
    // Map the speed from threshold
    pwm = (uint8_t)map(lower_threshold - speed, 0, lower_threshold, 0, 255);
    backward = true;
  } else {
    // Deadzone: Input is between 117 and 137
    pwm      = 0;
    forward  = false;
    backward = false;
  }

  // Write to hardware
  digitalWrite(MOTOR_FORWARD, forward ? HIGH : LOW);
  digitalWrite(MOTOR_BACKWARD, backward ? HIGH : LOW);
  analogWrite(MOTOR_SPEED, pwm);

  // Update the train speed variable
  if (forward || backward)
    train_speed = pwm;
  else
    train_speed = 0;
}
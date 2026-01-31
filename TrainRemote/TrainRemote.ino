/**
 * @file    TrainRemote.ino
 * @author  Luka Jacobsen (Prototyped by Jacq & Benjamin)
 * @brief   Remote control for LEGO model train.
 * @date    2026-01-09
 * * @details This file sends wireless packets to the controller that handles
 * the train. The remote has support for autopilot, headlights and manual speed.
 * * HARDWARECONNECTIONS:
 * - Pin 12: Transmit Wireless (Digital, TX)
 * - Pin 7:  Headlights        (Digital)
 * - Pin 8:  Autopilot         (Digital)
 * - Pin A0: Motor speed       (Analog, PWM)
 * - Pin 5:  LED Forward Indi  (Digital, PWM)
 * - Pin 6:  LED Backward Indi (Digital, PWM)
 */

// Include libraries to transmit data
#include <RH_ASK.h>
#include <SPI.h>

// Create transmitter driver
RH_ASK rf_driver;

// ========================================================================== //
// CONFIGURATION                                                              //
// ========================================================================== //

// -- Transmitter / Input Reading -- //
const int READ_TRANSMIT_DELAY = 20;   // (ms) Read and send once every 100ms

// -------- Speeds (0-255) --------- //
const uint8_t DEADZONE = 10;    // +- center

// ========================================================================== //
// PIN DEFINITIONS                                                            //
// ========================================================================== //

const uint8_t MOTOR_SPEED   = A0;
const uint8_t HEADLIGHT_PIN = 7;
const uint8_t AUTOPILOT_PIN = 8;
const uint8_t TRANSMIT_SEND = 1;
const uint8_t LED_FORWARD   = 5;
const uint8_t LED_BACKWARD  = 6;

// ========================================================================== //
// RUNTIME GLOBAL VARIABLES                                                   //
// ========================================================================== //

// ------- Last read values -------- //
uint8_t last_manual_speed    = 0;
uint8_t last_headlight_state = false;
uint8_t last_autopilot_state = false;

// ========================================================================== //
// LIFECYCLE FUNCTIONS                                                        //
// ========================================================================== //

/**
 * @brief Called once on startup by system.
 */
void setup() {
  // Start serial (UART) on baudrate 9600
  Serial.begin(9600);

  // Initialize transmitter driver
  rf_driver.init();

  // Set controller pin modes
  pinMode(MOTOR_SPEED, INPUT_PULLUP);
  pinMode(HEADLIGHT_PIN, INPUT_PULLUP);
  pinMode(AUTOPILOT_PIN, INPUT_PULLUP);

  // Set indicator pin modes
  pinMode(LED_FORWARD, OUTPUT);
  pinMode(LED_BACKWARD, OUTPUT);

  // Set transmitter send pin
  pinMode(TRANSMIT_SEND, OUTPUT);
}

/**
 * @brief Called every time the processor can.
 */
void loop() {
  readAndSendInputs();
  setIndicatorLEDs(last_manual_speed);
}

// ========================================================================== //
// Main Functions                                                             //
// ========================================================================== //

/**
 * @brief Read pins and transmit their data if changed since last read.
 */
void readAndSendInputs() {
  // Read values from pins
  uint8_t pot_value = (uint8_t)map(analogRead(MOTOR_SPEED), 0, 1023, 0, 255);
  bool headlights   = digitalRead(HEADLIGHT_PIN);
  bool autopilot    = digitalRead(AUTOPILOT_PIN);

  // Check if any values are updated
  if (
    pot_value     != last_manual_speed
    || headlights != last_headlight_state
    || autopilot  != last_autopilot_state
  ) {
    // Pack the values into 2 bytes
    uint8_t b1, b2;
    packPacket(pot_value, autopilot, headlights, b1, b2);

    // Send the data to the controllers buffer
    uint8_t data[2] = {b1, b2};
    rf_driver.send(data, 2);
    rf_driver.waitPacketSent();
  }

  // Update the values
  last_manual_speed    = pot_value;
  last_headlight_state = headlights;
  last_autopilot_state = autopilot;
}

// ========================================================================== //
// Helper Functions                                                           //
// ========================================================================== //

/**
 * @brief Packs speed, autopilot and headlight states into two bytes. Ready to
 * send over to train controller.
 * @param speed The speed the train should run at, only sat at manual control.
 * @param autopilot If true, autopilot will be enabled.
 * @param headlights If true, headlights will be enabled.
 * @param b1 Reference to the first byte.
 * @param b2 Reference to the second byte.
 */
void packPacket(uint8_t speed, bool autopilot, bool headlights, uint8_t &b1, uint8_t &b2) {
  static uint8_t check = 0;     // Static check value, used to control bits
  check = (check + 1) & 0x03;   // Rolling 2-bit check

  // Byte 1
  b1 = (check << 6) |           // Check bits
       (0 << 5) |               // Frame = First byte
       ((speed >> 3) & 0x1F);   // Speed, high bits

  // Byte 2
  b2 = (check << 6) |           // Same check bits
       (1 << 5) |               // Frame = Second byte
       ((speed & 0x07) << 2) |  // Speed, low bits
       ((autopilot & 1) << 1) | // Autopilot flag
       (headlights & 1);        // Headlights flag
}

/**
 * @brief Lights up indicators on the remote to give feedback on which
 * direction the user is attempting to drive.
 * @param speed The speed the train should run at. i.e indicators.
 */
void setIndicatorLEDs(uint8_t speed) {
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
  analogWrite(LED_FORWARD, forward ? pwm : LOW);
  analogWrite(LED_BACKWARD, backward ? pwm : LOW);
}

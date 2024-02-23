// Include necessary libraries for Arduino and servo control
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize Adafruit PWM Servo Driver using default I2C address (0x40) and Wire interface
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Define minimum and maximum pulse lengths for various servos to match their range of motion
#define SERVOMIN_PAN 107    // Minimum pulse length count for pan servo
#define SERVOMAX_PAN 437    // Maximum pulse length count for pan servo
#define SERVOMIN_TILT 90    // Minimum pulse length count for tilt servo
#define SERVOMAX_TILT 380   // Maximum pulse length count for tilt servo
#define SERVOMIN_ROLL 90    // Minimum pulse length count for roll servo
#define SERVOMAX_ROLL 356   // Maximum pulse length count for roll servo
#define SERVOMIN_TILT_2 90  // Minimum pulse length count for second tilt servo
#define SERVOMAX_TILT_2 500 // Maximum pulse length count for second tilt servo
#define SERVOMIN_ROLL_2 90  // Minimum pulse length count for second roll servo
#define SERVOMAX_ROLL_2 500 // Maximum pulse length count for second roll servo
#define SERVOMIN_CRUNCH 0   // Minimum pulse length count for crunch servo
#define SERVOMAX_CRUNCH 400 // Maximum pulse length count for crunch servo

// Define timing for servo control
#define USMIN 600      // Minimum microsecond length
#define USMAX 2400     // Maximum microsecond length
#define SERVO_FREQ 50  // Servo update frequency (~50Hz for analog servos)

// Define servo pin numbers on the PWM driver
#define PAN_PIN 0 
#define TILT_PIN 1
#define ROLL_PIN 2
#define CRUNCH_PIN 3
#define TILT_PIN_2 4
#define ROLL_PIN_2 5

// Initialize servo positions
int16_t panPos = 90;
int16_t tiltPos = 150;
int16_t rollPos = 35;
int16_t CrunchPos = 90;
int16_t tiltPos2 = 90;
int16_t rollPos2 = 120;

// Setup function to initialize serial communication and servo driver settings
void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for serial monitor to open (necessary for boards like Leonardo)

  Serial.println("8 channel Servo test!");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000); // Calibrate internal oscillator frequency for accurate PWM updates
  pwm.setPWMFreq(SERVO_FREQ); // Set PWM update frequency for analog servos
  delay(10);

  // Initial setting of PWM values for servos to their starting positions
  pwm.setPWM(PAN_PIN, 0, panPos);
  pwm.setPWM(TILT_PIN, 0, tiltPos);
  pwm.setPWM(ROLL_PIN, 0, rollPos);
  pwm.setPWM(TILT_PIN_2, 0, tiltPos2);
  pwm.setPWM(ROLL_PIN_2, 0, rollPos2);
  pwm.setPWM(CRUNCH_PIN, 0, CrunchPos);
}

// Function to set servo pulse width in seconds (e.g., for precise control)
void setServoPulse(uint8_t n, double pulse) {
  double pulselength = 1000000.0 / SERVO_FREQ; // Calculate microseconds per PWM period
  pulselength /= 4096.0; // 12-bit resolution
  pulse *= 1000000.0; // Convert input seconds to microseconds
  pulse /= pulselength; // Convert pulse to PWM bit width
  pwm.setPWM(n, 0, pulse);
}

// Main loop for reading serial input and controlling servos based on commands
void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read(); // Read input character from serial

    // Control servos based on input character commands
    switch (input) {
      // Pan (Yaw) control
      case 'a':
      case 'd': {
        panPos += (input == 'a') ? 10 : -10;
        panPos = constrain(panPos, SERVOMIN_PAN, SERVOMAX_PAN);
        Serial.print("panPos="); Serial.println(panPos);
        pwm.setPWM(PAN_PIN, 0, panPos);
        break;
      }
      // Tilt control
      case 'r':
      case 'f': {
        tiltPos += (input == '

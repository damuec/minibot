#include <ESP32Servo.h> // Include the ESP32-specific Servo library

// --- PINS FOR THE L298N (REAR DRIVE MOTOR) ---
const int motorENA = 13; // PWM pin for motor speed
const int motorIN1 = 12; // Direction pin 1
const int motorIN2 = 14; // Direction pin 2

// --- PINS FOR THE SERVO (FRONT STEERING) ---
const int servoPin = 15;

// --- SERVO CALIBRATION VARIABLES ---
// You MUST calibrate these values for your specific robot!
const int servoCenter = 92;
const int servoRightMax = 45;  // Servo PWM value for max right turn
const int servoLeftMax = 135;  // Servo PWM value for max left turn
const float max_steer_angle = 0.5; // Max steering angle in radians (e.g., 28.6 degrees)

Servo steeringServo; // Create a Servo object

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Robot Booted!"); // Print a message to confirm boot

  // SETUP L298N CONTROL PINS
  pinMode(motorIN1, OUTPUT);
  pinMode(motorIN2, OUTPUT);
  pinMode(motorENA, OUTPUT);

  // Stop the motor in a safe state
  digitalWrite(motorIN1, LOW);
  digitalWrite(motorIN2, LOW);
  analogWrite(motorENA, 0); // Set speed to 0

  steeringServo.attach(servoPin);
  centerSteering();
  delay(500);
  Serial.println("Setup complete. Ready for commands.");
}

void centerSteering() {
  steeringServo.write(servoCenter);
}

void loop() {
  // Listen for commands from the Raspberry Pi
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    // Example command: "T150 S0.25"
    int throttle = 0;
    float steering_angle = 0.0;

    // Parse the Throttle value
    int tIndex = command.indexOf('T');
    int sIndex = command.indexOf('S');
    if (tIndex != -1 && sIndex != -1) {
      String throttleStr = command.substring(tIndex + 1, sIndex);
      throttle = throttleStr.toInt();

      // Parse the Steering angle value
      String steeringStr = command.substring(sIndex + 1);
      steering_angle = steeringStr.toFloat();
    }

    // Call your moveRobot function with the parsed values
    moveRobot(throttle, steering_angle);
  }
}

// Updated moveRobot function to handle float steering angle
void moveRobot(int throttle, float steering_angle) {
  // 1. Convert the steering angle from radians to a servo PWM value
  // This requires calibration! You need to know what servo value corresponds to what angle.
  int servo_pwm = servoCenter + (int)(steering_angle * (servoRightMax - servoCenter) / max_steer_angle);
  servo_pwm = constrain(servo_pwm, servoLeftMax, servoRightMax);
  steeringServo.write(servo_pwm);

  // 2. Control the rear drive motor (same as before)
  if (throttle > 10) {
    digitalWrite(motorIN1, HIGH);
    digitalWrite(motorIN2, LOW);
    analogWrite(motorENA, throttle);
  } else if (throttle < -10) {
    digitalWrite(motorIN1, LOW);
    digitalWrite(motorIN2, HIGH);
    analogWrite(motorENA, -throttle);
  } else {
    digitalWrite(motorIN1, LOW);
    digitalWrite(motorIN2, LOW);
    analogWrite(motorENA, 0);
  }
}
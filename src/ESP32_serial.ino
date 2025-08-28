#include <ESP32Servo.h>

// --- PINS ---
const int MOTOR_IN1 = 12;
const int MOTOR_IN2 = 14;
const int MOTOR_ENA = 13;
const int SERVO_PIN = 15;

// --- Configuration & Calibration ---
const int SERVO_CENTER = 92;
const int SERVO_LEFT_MAX = 135;   // PWM for max left turn
const int SERVO_RIGHT_MAX = 45;   // PWM for max right turn
const float MAX_STEER_ANGLE = 0.5; // radians

Servo steering_servo;

void setup() {
  Serial.begin(115200);
  
  // Motor control setup
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);
  brake(); // Start in a safe state

  // Servo setup
  steering_servo.attach(SERVO_PIN);
  steering_servo.write(SERVO_CENTER);
  
  delay(2000); // Wait for everything to initialize
  Serial.println("ESP32 Ready: T<throttle> S<steering_angle>");
}

void loop() {
  // Listen for commands from Raspberry Pi
  if(Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
  }
}

void processCommand(String cmd) {
  // Example command: "T150 S0.25"
  int throttle = 0;
  float steering_angle = 0.0;
  
  // Find positions of T and S in the command string
  int t_index = cmd.indexOf('T');
  int s_index = cmd.indexOf('S');
  
  if(t_index != -1 && s_index != -1 && s_index > t_index) {
    // Extract throttle value (between T and S)
    String throttle_str = cmd.substring(t_index+1, s_index);
    throttle = throttle_str.toInt();
    
    // Extract steering angle (after S)
    String angle_str = cmd.substring(s_index+1);
    steering_angle = angle_str.toFloat();
    
    // Execute the command
    setSteering(steering_angle);
    setThrottle(throttle);
    
    // Optional: echo back for confirmation
    // Serial.println("OK");
  }
}

void setSteering(float angle) {
  // Convert radians to PWM value
  int pwm_value = SERVO_CENTER + (int)(angle * (SERVO_RIGHT_MAX - SERVO_CENTER) / MAX_STEER_ANGLE);
  pwm_value = constrain(pwm_value, SERVO_RIGHT_MAX, SERVO_LEFT_MAX);
  steering_servo.write(pwm_value);
}

void setThrottle(int throttle) {
  if(throttle > 10) { // Forward
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_ENA, throttle);
  } 
  else if(throttle < -10) { // Backward
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_ENA, -throttle);
  } 
  else { // Stop
    brake();
  }
}

void brake() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_ENA, 0);
}
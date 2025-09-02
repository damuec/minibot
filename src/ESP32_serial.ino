#include <ESP32Servo.h>
#include <math.h>

// --- Hardware Configuration ---
const int MOTOR_IN1 = 12;
const int MOTOR_IN2 = 14;
const int SERVO_PIN = 15;

// --- PWM Configuration ---
const int MOTOR_PWM_CHANNEL_1 = 0;
const int MOTOR_PWM_CHANNEL_2 = 1;
const int SERVO_PWM_CHANNEL = 2;
const int PWM_FREQUENCY = 1000;
const int PWM_RESOLUTION = 8;

// --- Robot Physical Parameters (UPDATE THESE VALUES) ---
const float WHEEL_RADIUS = 0.05;        // Wheel radius in meters (5cm)
const float WHEEL_BASE = 0.3;           // Distance between wheels in meters (30cm)
const float GEAR_RATIO = 20.0;          // Motor gear ratio
const float MAX_MOTOR_RPM = 550.0;      // Maximum RPM of your motor
const float BATTERY_VOLTAGE = 12.0;     // Operating voltage

// --- Servo Calibration ---
const int SERVO_CENTER = 90;
const int SERVO_LEFT_MAX = 120;
const int SERVO_RIGHT_MAX = 60;
const float MAX_STEER_ANGLE = 0.5;   

// --- Serial Configuration ---
const int SERIAL_BAUD_RATE = 115200;
const int SERIAL_TIMEOUT = 10;          // ms
const int BUFFER_SIZE = 32;

// --- Performance Optimization ---
const int ODOMETRY_UPDATE_MS = 100;     // Odometry update interval
const int COMMAND_TIMEOUT_MS = 500;     // Safety timeout

// --- Calculated Constants ---
const float MAX_LINEAR_VELOCITY = (MAX_MOTOR_RPM * 2 * M_PI * WHEEL_RADIUS) / (60 * GEAR_RATIO);
const float PWM_TO_VELOCITY = MAX_LINEAR_VELOCITY / 255.0;

Servo steering_servo;
unsigned long last_command_time = 0;
unsigned long last_odometry_time = 0;

// Current command values
int current_throttle = 0;
float current_steering = 0.0;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.setTimeout(SERIAL_TIMEOUT);
  
  // Configure motor control with LEDC
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  
  ledcSetup(MOTOR_PWM_CHANNEL_1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(MOTOR_PWM_CHANNEL_2, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_IN1, MOTOR_PWM_CHANNEL_1);
  ledcAttachPin(MOTOR_IN2, MOTOR_PWM_CHANNEL_2);
  
  // Initialize in safe state
  brake();
  
  // Configure servo
  steering_servo.attach(SERVO_PIN);
  steering_servo.write(SERVO_CENTER);
  
  // Wait for stabilization
  delay(2000);
  Serial.println("ESP32_READY");
  
  // Print robot configuration
  Serial.print("MAX_LINEAR_VELOCITY:");
  Serial.println(MAX_LINEAR_VELOCITY);
}

void loop() {
  // Process incoming commands
  if (Serial.available() > 0) {
    char buffer[BUFFER_SIZE];
    int bytes_read = Serial.readBytesUntil('\n', buffer, BUFFER_SIZE - 1);
    buffer[bytes_read] = '\0'; // Null-terminate
    
    if (bytes_read > 0) {
      processCommand(buffer);
      last_command_time = millis();
    }
  }
  
  // Safety timeout - stop if no commands received
  if (millis() - last_command_time > COMMAND_TIMEOUT_MS) {
    brake();
    current_throttle = 0;
    current_steering = 0.0;
  }
  
  // Update odometry and send data
  if (millis() - last_odometry_time > ODOMETRY_UPDATE_MS) {
    sendOdometry();
    last_odometry_time = millis();
  }
}

void processCommand(const char* command) {
  // Expected format: "T<throttle>,S<steering>"
  int throttle = 0;
  float steering = 0.0;
  
  // Parse command efficiently
  const char* throttle_ptr = strchr(command, 'T');
  const char* steering_ptr = strchr(command, 'S');
  
  if (throttle_ptr && steering_ptr && steering_ptr > throttle_ptr) {
    throttle = atoi(throttle_ptr + 1);
    steering = atof(steering_ptr + 1);
    
    setThrottle(throttle);
    setSteering(steering);
    
    // Store current values for odometry calculation
    current_throttle = throttle;
    current_steering = steering;
    
    // Echo for verification
    Serial.print("OK:T");
    Serial.print(throttle);
    Serial.print(",S");
    Serial.println(steering);
  }
}

void setSteering(float angle) {
  // Constrain and convert angle to PWM
  angle = constrain(angle, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
  int pwm_value = SERVO_CENTER + (int)(angle * (SERVO_RIGHT_MAX - SERVO_CENTER) / MAX_STEER_ANGLE);
  pwm_value = constrain(pwm_value, SERVO_RIGHT_MAX, SERVO_LEFT_MAX);
  steering_servo.write(pwm_value);
}

void setThrottle(int throttle) {
  throttle = constrain(throttle, -255, 255);
  
  if (throttle > 10) { // Forward
    ledcWrite(MOTOR_PWM_CHANNEL_1, throttle);
    ledcWrite(MOTOR_PWM_CHANNEL_2, 0);
  } else if (throttle < -10) { // Backward
    ledcWrite(MOTOR_PWM_CHANNEL_1, 0);
    ledcWrite(MOTOR_PWM_CHANNEL_2, -throttle);
  } else { // Stop
    brake();
  }
}

void brake() {
  ledcWrite(MOTOR_PWM_CHANNEL_1, 0);
  ledcWrite(MOTOR_PWM_CHANNEL_2, 0);
}

void sendOdometry() {
  // Since we don't have encoders, we'll estimate based on commands
  float linear_vel = current_throttle * PWM_TO_VELOCITY;
  float angular_vel = 0.0;
  
  // For a car-like steering, angular velocity = linear_vel * tan(steering_angle) / wheel_base
  if (abs(current_steering) > 0.01 && abs(linear_vel) > 0.01) {
    angular_vel = linear_vel * tan(current_steering) / WHEEL_BASE;
  }
  
  // Send odometry data
  Serial.print("ODOM:");
  Serial.print(0.0);  // x position (not available without encoders)
  Serial.print(",");
  Serial.print(0.0);  // y position (not available without encoders)
  Serial.print(",");
  Serial.print(0.0);  // theta orientation (not available without encoders)
  Serial.print(",");
  Serial.print(linear_vel);
  Serial.print(",");
  Serial.println(angular_vel);
}
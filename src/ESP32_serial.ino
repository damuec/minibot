#include <ESP32Servo.h>
#include <math.h>

// --- Hardware Configuration for L298N ---
const int MOTOR_IN1 = 16;  
const int MOTOR_IN2 = 17;
const int SERVO_PIN = 15;

// --- PWM Configuration ---
// Using two PWM channels for motor control (L298N uses direction pins for speed control)
const int MOTOR_PWM_CHANNEL_1 = 6;  
const int MOTOR_PWM_CHANNEL_2 = 5;  
const int SERVO_PWM_CHANNEL = 2;
const int PWM_FREQUENCY = 1000;
const int PWM_RESOLUTION = 8;

// --- Robot Physical Parameters ---
const float WHEEL_RADIUS = 0.05;   // cm     
const float WHEEL_BASE = 0.3;     // cm  
const float GEAR_RATIO = 20.0;      // cm 
const float MAX_MOTOR_RPM = 550.0;    // rpm

// --- Servo Calibration ---
const int SERVO_CENTER = 90;
const int SERVO_LEFT_MAX = 120;
const int SERVO_RIGHT_MAX = 60;
const float MAX_STEER_ANGLE = 0.5;   

// --- Serial Configuration ---
const int SERIAL_BAUD_RATE = 115200;
const int SERIAL_TIMEOUT = 10;
const int BUFFER_SIZE = 32;

// --- Performance Optimization ---
const int COMMAND_TIMEOUT_MS = 500;    
Servo steering_servo;

unsigned long last_command_time = 0;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.setTimeout(SERIAL_TIMEOUT);
  
  // Configure L298N motor control pins
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  
  // Setup PWM channels for L298N direction pins
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
}

void loop() {
  // Process incoming commands
  if (Serial.available() > 0) {
    char buffer[BUFFER_SIZE];
    int bytes_read = Serial.readBytesUntil('\n', buffer, BUFFER_SIZE - 1);
    buffer[bytes_read] = '\0'; 

    if (bytes_read > 0) {
      processCommand(buffer);
      last_command_time = millis();
    }
  }
  
  // Safety timeout - stop if no commands received
  if (millis() - last_command_time > COMMAND_TIMEOUT_MS) {
    brake();
  }
}

void processCommand(const char* command) {
  // Expected format: "T<throttle>,S<steering>"
  int throttle = 0;
  float steering = 0.0;
  
  const char* throttle_ptr = strchr(command, 'T');
  const char* steering_ptr = strchr(command, 'S');
  
  if (throttle_ptr && steering_ptr && steering_ptr > throttle_ptr) {
    throttle = atoi(throttle_ptr + 1);
    steering = atof(steering_ptr + 1);
    
    setThrottle(throttle);
    setSteering(steering);
    
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
  
  if (throttle > 10) {
    ledcWrite(MOTOR_PWM_CHANNEL_1, throttle);
    ledcWrite(MOTOR_PWM_CHANNEL_2, 0);
  } else if (throttle < -10) { 
    ledcWrite(MOTOR_PWM_CHANNEL_1, 0);
    ledcWrite(MOTOR_PWM_CHANNEL_2, -throttle);
  } else { 
    brake();
  }
}

void brake() {
  ledcWrite(MOTOR_PWM_CHANNEL_1, 0);
  ledcWrite(MOTOR_PWM_CHANNEL_2, 0);
}
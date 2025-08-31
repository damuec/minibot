#include <ESP32Servo.h>

// --- PINS ---
const int MOTOR_IN1 = 12;
const int MOTOR_IN2 = 14;
const int SERVO_PIN = 15;

// --- PWM Configuration ---
const int MOTOR_PWM_CHANNEL = 0;
const int PWM_FREQUENCY = 1000;
const int PWM_RESOLUTION = 8;

// --- Configuration & Calibration ---
const int SERVO_CENTER = 92;
const int SERVO_LEFT_MAX = 135;
const int SERVO_RIGHT_MAX = 45;
const float MAX_STEER_ANGLE = 0.5;

Servo steering_servo;

void setup() {
  Serial.begin(115200);
  
  // Motor control setup with LEDC
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  
  // Configure LEDC for motor control
  ledcSetup(MOTOR_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_IN1, MOTOR_PWM_CHANNEL);
  
  brake(); // Start in a safe state

  // Servo setup
  steering_servo.attach(SERVO_PIN);
  steering_servo.write(SERVO_CENTER);
  
  delay(2000);
  Serial.println("ESP32 Ready: T<throttle> S<steering_angle>");
}

void loop() {
  // Listen for commands from Raspberry Pi
  if(Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
    
    // Echo the command back for verification
    Serial.print("ECHO:");
    Serial.println(command);
  }
  
  // Send simulated odometry data back to Raspberry Pi
  static unsigned long last_odom_time = 0;
  if(millis() - last_odom_time > 100) {
    float linear_vel = 0.1;  // m/s
    float angular_vel = 0.05; // rad/s
    
    Serial.print(linear_vel);
    Serial.print(",");
    Serial.println(angular_vel);
    
    last_odom_time = millis();
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
  }
}

void setSteering(float angle) {
  // Convert radians to PWM value
  int pwm_value = SERVO_CENTER + (int)(angle * (SERVO_RIGHT_MAX - SERVO_CENTER) / MAX_STEER_ANGLE);
  pwm_value = constrain(pwm_value, SERVO_RIGHT_MAX, SERVO_LEFT_MAX);
  steering_servo.write(pwm_value);
}

void setThrottle(int throttle) {
  throttle = constrain(throttle, -255, 255);
  
  if(throttle > 10) { // Forward
    ledcWrite(MOTOR_PWM_CHANNEL, throttle);
    digitalWrite(MOTOR_IN2, LOW);
  } 
  else if(throttle < -10) { // Backward
    ledcWrite(MOTOR_PWM_CHANNEL, -throttle);
    digitalWrite(MOTOR_IN1, LOW);
  } 
  else { // Stop
    brake();
  }
}

void brake() {
  ledcWrite(MOTOR_PWM_CHANNEL, 0);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}
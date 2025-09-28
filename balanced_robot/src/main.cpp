#include <Arduino.h>
#include "imu.h"
#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi Configuration
const char* ssid = "CKVNLLP";
const char* password = "jamaludheen6026";
WiFiUDP udp;
unsigned int localPort = 8888;
char packetBuffer[255];

#define enableMotor1 25  // PWM capable pin
#define motor1Pin1 32
#define motor1Pin2 33

// Motor 2 pins (mapped to ESP32 GPIO)
#define motor2Pin1 27
#define motor2Pin2 14
#define enableMotor2 26  // PWM capable pin

// PID - Optimized for self-balancing
float Kp = 10.0;   // Proportional gain - increased for faster response
float Ki = 0.0;  // Integral gain - eliminates steady-state error
float Kd = 0.0;    // Derivative gain - provides damping

// System limits
const float Max_Speed = 255.0;  // PWM range for L298N (0-255)
const float Max_Angle = 45.0;
const float Max_Integral = 50.0;
const float Dead_Zone = 0.5;

// === MOTOR CONTROL VARIABLES === //
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
const int PWM_FREQUENCY = 1000;  // 1kHz PWM frequency
const int PWM_RESOLUTION = 8;    // 8-bit resolution (0-255)
const int PWM_CHANNEL_1 = 0;     // LEDC channel for enableMotor1
const int PWM_CHANNEL_2 = 1;     // LEDC channel for enableMotor2

// === PID VARIABLES === //
float previous_error = 0.0;
float integral = 0.0;
float previous_angle = 0.0;
float angle_velocity = 0.0;
unsigned long previous_time = 0;
bool robot_active = false;

// === ADVANCED CONTROL VARIABLES === //
float velocity_setpoint = 0.0;  // For future expansion (remote control)
float Kv = 5.0;  // Velocity feedback gain

// === MOTOR CONTROL FUNCTIONS === //
void setMotorSpeed(int leftSpeed, int rightSpeed) {
    // Constrain speeds to valid PWM range
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    // Motor 1 Control
    if (leftSpeed > 0) {
        // Forward direction
        digitalWrite(motor1Pin1, HIGH);
        digitalWrite(motor1Pin2, LOW);
        ledcWrite(PWM_CHANNEL_1, leftSpeed);
    } else if (leftSpeed < 0) {
        // Reverse direction
        digitalWrite(motor1Pin1, LOW);
        digitalWrite(motor1Pin2, HIGH);
        ledcWrite(PWM_CHANNEL_1, -leftSpeed);
    } else {
        // Stop motor
        digitalWrite(motor1Pin1, LOW);
        digitalWrite(motor1Pin2, LOW);
        ledcWrite(PWM_CHANNEL_1, 0);
    }
    
    // Motor 2 Control
    if (rightSpeed > 0) {
        // Forward direction
        digitalWrite(motor2Pin1, HIGH);
        digitalWrite(motor2Pin2, LOW);
        ledcWrite(PWM_CHANNEL_2, rightSpeed);
    } else if (rightSpeed < 0) {
        // Reverse direction
        digitalWrite(motor2Pin1, LOW);
        digitalWrite(motor2Pin2, HIGH);
        ledcWrite(PWM_CHANNEL_2, -rightSpeed);
    } else {
        // Stop motor
        digitalWrite(motor2Pin1, LOW);
        digitalWrite(motor2Pin2, LOW);
        ledcWrite(PWM_CHANNEL_2, 0);
    }
}

void stopMotors() {
    setMotorSpeed(0, 0);
}
void setupWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
  
  udp.begin(localPort);
  Serial.println("UDP server started");
}

void handleUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    
    // Parse PID values (format: "Kp:1.2,Ki:0.5,Kd:0.1")
    char* cmd = strtok(packetBuffer, ",");
    while (cmd != NULL) {
      if (strncmp(cmd, "Kp:", 3) == 0) Kp = atof(cmd + 3);
      else if (strncmp(cmd, "Ki:", 3) == 0) Ki = atof(cmd + 3);
      else if (strncmp(cmd, "Kd:", 3) == 0) Kd = atof(cmd + 3);
      cmd = strtok(NULL, ",");
    }
    
    // Send back telemetry data
    char reply[100];
    snprintf(reply, 100, "Angle:%.2f,Kp:%.2f,Ki:%.2f,Kd:%.2f", 
             getPitchAngle(), Kp, Ki, Kd);
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write((uint8_t*)reply, strlen(reply));
    udp.endPacket();
  }
}
// === SETUP === //
void setup() {
    Serial.begin(115200);
    setupWiFi();
    // Initialize L298N motor driver pins
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);
    
    // Configure LEDC PWM for motor speed control
    ledcSetup(PWM_CHANNEL_1, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_2, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(enableMotor1, PWM_CHANNEL_1);
    ledcAttachPin(enableMotor2, PWM_CHANNEL_2);
    
    // Stop motors initially
    stopMotors();
    
    initIMU();
    calibrateIMU();
    delay(1000);
    
    updateIMU();
    float initial_angle = getPitchAngle();
    
    if (abs(initial_angle) < 10.0) {
        robot_active = true;
        previous_time = millis();
        Serial.println("Robot activated!");
    } else {
        Serial.println("Robot not upright!");
    }
}

// === MAIN LOOP === //
void loop() {
    handleUDP();
    updateIMU();
    float angle = getPitchAngle();
    
    if (abs(angle) > Max_Angle) {
        if (robot_active) {
            Serial.println("Robot fallen! Stopping motors.");
            stopMotors();
            robot_active = false;
        }
        return;
    }
    
    if (!robot_active) return;
    
    unsigned long current_time = millis();
    float dt = (current_time - previous_time) / 1000.0;
    dt = constrain(dt, 0.001, 0.1);
    
    // Calculate angle velocity (angular rate from angle change)
    angle_velocity = (angle - previous_angle) / dt;
    previous_angle = angle;
    
    // Setpoint adjustment based on velocity (allows robot to lean into motion)
    float dynamic_setpoint = velocity_setpoint * 0.1;  // Small lean for movement
    float error = dynamic_setpoint - angle;
    
    // Apply dead zone only for very small errors
    if (abs(error) < Dead_Zone) error = 0.0;
    
    // PID calculations
    integral += error * dt;
    integral = constrain(integral, -Max_Integral, Max_Integral);
    
    float derivative = (error - previous_error) / dt;
    
    // Enhanced PID with velocity feedback
    float pid_output = Kp * error + Ki * integral + Kd * derivative;
    float velocity_feedback = Kv * angle_velocity;
    float output = pid_output - velocity_feedback;  // Velocity feedback opposes motion
    
    previous_error = error;
    previous_time = current_time;
    
    output = constrain(output, -Max_Speed, Max_Speed);
    
    // Convert PID output to motor speeds
    // For balancing: both motors move in same direction to correct tilt
    int motorSpeed = (int)output;
    setMotorSpeed(motorSpeed, -motorSpeed);  // Opposite directions for balancing
    
    static unsigned long last_print = 0;
    if (current_time - last_print > 100) {
        Serial.print("Kp: "); Serial.print(Kp);
        Serial.print(" | Ki: "); Serial.print(Ki);
        Serial.print(" | Kd: "); Serial.print(Kd);
        Serial.print(" | Angle: "); Serial.print(angle, 2);
        Serial.print("° | Output: "); Serial.println(output, 1);
        last_print = current_time;
    }
}


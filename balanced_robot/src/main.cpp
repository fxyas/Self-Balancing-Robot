#include <Arduino.h>
#include "imu.h"
#include <WiFi.h>
#include <WiFiUdp.h>

// ===== Function Declarations =====
void setupMotors();
void setMotorSpeed(int leftSpeed, int rightSpeed);
void stopMotors();
void setupWiFi();
void handleUDP();
float calculatePID(float angle, float dt);

// ===== WiFi Configuration =====
const char* ssid = "CKVNLLP";
const char* password = "jamaludheen6026";
WiFiUDP udp;
unsigned int localPort = 8888;
char packetBuffer[255];

// ===== Motor Pin Configuration =====
// Motor 1 (Left Motor)
#define enableMotor1 25  // PWM pin
#define motor1Pin1 32
#define motor1Pin2 33

// Motor 2 (Right Motor)
#define enableMotor2 26  // PWM pin
#define motor2Pin1 27
#define motor2Pin2 14

// ===== PID Controller Gains =====
// Start with conservative values and tune incrementally
float Kp = 15.0;   // Proportional: Main balancing force
float Ki = 0.5;    // Integral: Corrects drift over time
float Kd = 0.8;    // Derivative: Dampens oscillations

// ===== System Constants =====
const float MAX_MOTOR_SPEED = 255.0;  // Maximum PWM value
const float MAX_ANGLE = 35.0;         // Robot falls beyond this angle
const float MAX_INTEGRAL = 100.0;     // Prevent integral windup
const float ANGLE_SETPOINT = 0.0;     // Target angle (0° = upright)

// ===== PWM Configuration =====
const int PWM_FREQUENCY = 5000;  // 5kHz for smooth motor control
const int PWM_RESOLUTION = 8;    // 8-bit (0-255)
const int PWM_CHANNEL_1 = 0;     
const int PWM_CHANNEL_2 = 1;     

// ===== PID Variables =====
float previous_error = 0.0;
float integral = 0.0;
unsigned long previous_time = 0;
bool robot_active = false;

// ===== Timing =====
unsigned long last_print_time = 0;
const int PRINT_INTERVAL = 100;  // Print debug every 100ms

// ===== MOTOR CONTROL FUNCTIONS =====
void stopMotors() {
    setMotorSpeed(0, 0);
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
    // Constrain speeds
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    // Left Motor Control
    if (leftSpeed > 0) {
        digitalWrite(motor1Pin1, HIGH);
        digitalWrite(motor1Pin2, LOW);
        ledcWrite(PWM_CHANNEL_1, leftSpeed);
    } else if (leftSpeed < 0) {
        digitalWrite(motor1Pin1, LOW);
        digitalWrite(motor1Pin2, HIGH);
        ledcWrite(PWM_CHANNEL_1, -leftSpeed);
    } else {
        digitalWrite(motor1Pin1, LOW);
        digitalWrite(motor1Pin2, LOW);
        ledcWrite(PWM_CHANNEL_1, 0);
    }
    
    // Right Motor Control
    if (rightSpeed > 0) {
        digitalWrite(motor2Pin1, HIGH);
        digitalWrite(motor2Pin2, LOW);
        ledcWrite(PWM_CHANNEL_2, rightSpeed);
    } else if (rightSpeed < 0) {
        digitalWrite(motor2Pin1, LOW);
        digitalWrite(motor2Pin2, HIGH);
        ledcWrite(PWM_CHANNEL_2, -rightSpeed);
    } else {
        digitalWrite(motor2Pin1, LOW);
        digitalWrite(motor2Pin2, LOW);
        ledcWrite(PWM_CHANNEL_2, 0);
    }
}

void setupMotors() {
    // Configure motor direction pins
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);
    
    // Configure PWM channels
    ledcSetup(PWM_CHANNEL_1, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_2, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(enableMotor1, PWM_CHANNEL_1);
    ledcAttachPin(enableMotor2, PWM_CHANNEL_2);
    
    // Initialize motors stopped
    stopMotors();
}

// ===== WIFI FUNCTIONS =====
void setupWiFi() {
    Serial.println("\n===== WiFi Setup =====");
    Serial.print("Connecting to: ");
    Serial.println(ssid);
    
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✓ WiFi Connected!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        
        udp.begin(localPort);
        Serial.print("UDP Port: ");
        Serial.println(localPort);
    } else {
        Serial.println("\n✗ WiFi Failed - Running standalone");
    }
}

void handleUDP() {
    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(packetBuffer, 255);
        if (len > 0) packetBuffer[len] = 0;
        
        // Parse commands: "Kp:15.0,Ki:0.5,Kd:0.8"
        char* token = strtok(packetBuffer, ",");
        while (token != NULL) {
            if (strncmp(token, "Kp:", 3) == 0) {
                Kp = atof(token + 3);
                Serial.print("Updated Kp: "); Serial.println(Kp);
            }
            else if (strncmp(token, "Ki:", 3) == 0) {
                Ki = atof(token + 3);
                Serial.print("Updated Ki: "); Serial.println(Ki);
            }
            else if (strncmp(token, "Kd:", 3) == 0) {
                Kd = atof(token + 3);
                Serial.print("Updated Kd: "); Serial.println(Kd);
            }
            else if (strncmp(token, "RESET", 5) == 0) {
                integral = 0;  // Reset integral term
                Serial.println("PID Reset!");
            }
            token = strtok(NULL, ",");
        }
        
        // Send telemetry back
        char reply[128];
        snprintf(reply, sizeof(reply), 
                "Angle:%.2f,Kp:%.2f,Ki:%.2f,Kd:%.2f,Active:%d", 
                getPitchAngle(), Kp, Ki, Kd, robot_active);
        
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.write((uint8_t*)reply, strlen(reply));
        udp.endPacket();
    }
}

// ===== PID CONTROLLER =====
float calculatePID(float angle, float dt) {
    // Calculate error (how far from upright)
    float error = ANGLE_SETPOINT - angle;
    
    // Proportional term
    float P = Kp * error;
    
    // Integral term (accumulated error over time)
    integral += error * dt;
    integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);
    float I = Ki * integral;
    
    // Derivative term (rate of error change)
    float derivative = (error - previous_error) / dt;
    float D = Kd * derivative;
    
    // Store for next iteration
    previous_error = error;
    
    // Combined PID output
    float output = P + I + D;
    
    return output;
}

// ===== SETUP =====
void setup() {
    Serial.begin(115200);
    delay(100);
    
    Serial.println("\n====================================");
    Serial.println("   Self-Balancing Robot v2.0");
    Serial.println("====================================");
    
    // Initialize components
    setupMotors();
    setupWiFi();
    
    // Initialize and calibrate IMU
    Serial.println("\n===== IMU Initialization =====");
    initIMU();
    //calibrateIMU();
    imu_calibrated = true; 
    // Wait for robot to be placed upright
    Serial.println("\n===== Startup Check =====");
    Serial.println("Place robot upright to begin...");
    
    delay(1000);
    updateIMU();
    float initial_angle = getPitchAngle();
    
    if (abs(initial_angle) < 10.0) {
        robot_active = true;
        previous_time = millis();
        integral = 0;
        Serial.println("✓ Robot ACTIVE - Balancing started!");
        Serial.println("====================================\n");
    } else {
        Serial.println("✗ Robot tilted too much!");
        Serial.print("Current angle: ");
        Serial.print(initial_angle);
        Serial.println("° (needs < 10°)");
    }
}

// ===== MAIN LOOP =====
void loop() {
    // Handle WiFi commands if connected
    if (WiFi.status() == WL_CONNECTED) {
        handleUDP();
    }
    
    // Update IMU readings
    updateIMU();
    float current_angle = getPitchAngle();
    
    // Check if robot has fallen
    if (abs(current_angle) > MAX_ANGLE) {
        if (robot_active) {
            Serial.println("\n⚠ ROBOT FALLEN - Motors stopped");
            Serial.print("Final angle: ");
            Serial.print(current_angle);
            Serial.println("°");
            stopMotors();
            robot_active = false;
            integral = 0;  // Reset integral
        }
        delay(100);  // Slow down loop when fallen
        return;
    }
    
    // Check if robot should reactivate (picked up and placed upright)
    if (!robot_active && abs(current_angle) < 10.0) {
        robot_active = true;
        previous_time = millis();
        integral = 0;
        Serial.println("\n✓ Robot REACTIVATED");
    }
    
    // Skip control if not active
    if (!robot_active) {
        delay(100);
        return;
    }
    
    // Calculate time delta
    unsigned long current_time = millis();
    float dt = (current_time - previous_time) / 1000.0;
    
    // Prevent irregular time steps
    if (dt < 0.001 || dt > 0.1) {
        previous_time = current_time;
        return;
    }
    
    // Run PID controller
    float motor_output = calculatePID(current_angle, dt);
    
    // Constrain output to motor limits
    motor_output = constrain(motor_output, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    
    // Apply dead zone for noise reduction (optional)
    if (abs(motor_output) < 10) {
        motor_output = 0;
    }
    
    // Set motor speeds
    // Both motors move in same direction to correct tilt
    // Negative because of motor mounting orientation
    int motor_speed = (int)motor_output;
    setMotorSpeed(motor_speed, -motor_speed);
    
    // Update timing
    previous_time = current_time;
    
    // Debug output
    if (current_time - last_print_time > PRINT_INTERVAL) {
        Serial.print("Angle: ");
        Serial.print(current_angle, 1);
        Serial.print("° | PID: ");
        Serial.print(motor_output, 0);
        Serial.print(" | Kp:");
        Serial.print(Kp, 1);
        Serial.print(" Ki:");
        Serial.print(Ki, 1);
        Serial.print(" Kd:");
        Serial.print(Kd, 1);
        Serial.println();
        
        last_print_time = current_time;
    }
    
    // Small delay for loop stability
    delay(5);  // ~200Hz update rate
}
#include <Arduino.h>
#include <AccelStepper.h>
#include "imu.h"

// === CONFIGURATION === //
#define DIR_L 5
#define STEP_L 2
#define DIR_R 6
#define STEP_R 3
#define ENABLE_PIN 8

// PID Constants (start conservative, then tune)
const float Kp = 5.0;      // Reduced from 30
const float Ki = 0.0;       // Reduced from 0.6
const float Kd = 0.0;       // Reduced from 9

// System limits
const float Max_Speed = 200.0;
const float Max_Angle = 45.0;        // Shut down if tilted beyond this
const float Max_Integral = 50.0;     // Integral windup protection
const float Dead_Zone = 0.5;         // Ignore errors smaller than this

// === STEPPERS === //
AccelStepper motorLeft(AccelStepper::DRIVER, STEP_L, DIR_L);
AccelStepper motorRight(AccelStepper::DRIVER, STEP_R, DIR_R);

// === PID VARIABLES === //
float previous_error = 0.0;
float integral = 0.0;
unsigned long previous_time = 0;
bool robot_active = false;

// === SETUP === //
void setup() {
    Serial.begin(115200);
    
    // Initialize pins
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);  // Disable motors initially
    
    // Configure steppers
    motorLeft.setMaxSpeed(Max_Speed);
    motorRight.setMaxSpeed(Max_Speed);
    motorLeft.setAcceleration(3000);   // Add acceleration limiting
    motorRight.setAcceleration(3000);
    
    // Initialize IMU
    initIMU();
    calibrateIMU();
    
    // Wait for IMU to stabilize
    delay(1000);
    
    // Check if robot is upright before starting
    updateIMU();
    float initial_angle = getPitchAngle();
    
    if (abs(initial_angle) < 10.0) {  // Within 10 degrees of upright
        digitalWrite(ENABLE_PIN, LOW);  // Enable motors
        robot_active = true;
        previous_time = millis();
        Serial.println("Robot activated! Starting balance control...");
        Serial.print("Initial angle: ");
        Serial.println(initial_angle);
    } else {
        Serial.println("Robot not upright! Place upright and reset.");
        Serial.print("Current angle: ");
        Serial.println(initial_angle);
    }
}

// === MAIN LOOP === //
void loop() {
    // Update IMU
    updateIMU();
    float angle = getPitchAngle();
    
    // Safety check - shut down if fallen over
    if (abs(angle) > Max_Angle) {
        if (robot_active) {
            Serial.println("Robot fallen! Stopping motors.");
            digitalWrite(ENABLE_PIN, HIGH);  // Disable motors
            robot_active = false;
        }
        return;  // Exit loop, don't run PID
    }
    
    // Only run control if robot is active
    if (!robot_active) {
        return;
    }
    
    // === TIME-BASED PID COMPUTATION === //
    unsigned long current_time = millis();
    float dt = (current_time - previous_time) / 1000.0;  // Convert to seconds
    dt = constrain(dt, 0.001, 0.1);  // Constrain dt to reasonable range
    
    float setpoint = 0.0;  // Target angle (upright)
    float error = setpoint - angle;
    
    // Dead zone - ignore very small errors to reduce jitter
    if (abs(error) < Dead_Zone) {
        error = 0.0;
    }
    
    // Integral with windup protection
    integral += error * dt;
    integral = constrain(integral, -Max_Integral, Max_Integral);
    
    // Derivative
    float derivative = (error - previous_error) / dt;
    
    // PID output
    float output = Kp * error + Ki * integral + Kd * derivative;
    
    // Update for next iteration
    previous_error = error;
    previous_time = current_time;
    
    // Clamp motor speed
    output = constrain(output, -Max_Speed, Max_Speed);
    
    // === DRIVE MOTORS === //
    // Note: You may need to reverse one motor direction based on your wiring
    motorLeft.setSpeed(output);
    motorRight.setSpeed(-output);  // Verify this direction experimentally
    
    motorLeft.runSpeed();
    motorRight.runSpeed();
    
    // Debug output (comment out for better performance)
    static unsigned long last_print = 0;
    if (current_time - last_print > 100) {  // Print every 100ms
        Serial.print("Angle: ");
        Serial.print(angle, 2);
        Serial.print("° | Error: ");
        Serial.print(error, 2);
        Serial.print("° | Output: ");
        Serial.print(output, 1);
        Serial.print(" | I: ");
        Serial.println(integral, 2);
        last_print = current_time;
    }
    
    // Small delay for stability (optional)
    delay(2);
}

// === HELPER FUNCTIONS === //
void emergencyStop() {
    digitalWrite(ENABLE_PIN, HIGH);  // Disable motors
    robot_active = false;
    motorLeft.setSpeed(0);
    motorRight.setSpeed(0);
    Serial.println("EMERGENCY STOP!");
}

// Call this function to restart balancing (after manual uprighting)
void restartBalancing() {
    updateIMU();
    float angle = getPitchAngle();
    
    if (abs(angle) < 10.0) {
        // Reset PID variables
        integral = 0.0;
        previous_error = 0.0;
        previous_time = millis();
        
        // Enable system
        digitalWrite(ENABLE_PIN, LOW);
        robot_active = true;
        Serial.println("Balancing restarted!");
    } else {
        Serial.println("Robot not upright enough to restart.");
    }
}
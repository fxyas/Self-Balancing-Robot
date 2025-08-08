#include "stepper_test.h"

// Pin definitions for Motor 1 (Left)
#define MOTOR1_STEP_PIN 2
#define MOTOR1_DIR_PIN 5
#define MOTOR1_ENABLE_PIN 8

// Pin definitions for Motor 2 (Right)
#define MOTOR2_STEP_PIN 3
#define MOTOR2_DIR_PIN 6
#define MOTOR2_ENABLE_PIN 8

// Create stepper motor objects
AccelStepper motor1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);

// Motor parameters
const float MAX_SPEED = 1500.0;      // Maximum steps per second
const float ACCELERATION = 500.0;    // Steps per second squared
const int STEPS_PER_REV = 200;       // Typical for 1.8° stepper motors

// Control variables
float motorSpeed = 0;                // Current motor speed (-MAX_SPEED to +MAX_SPEED)
bool motorsEnabled = true;


// Function declarations
void handleSerialCommands();
void runMotors();
void printStatus();
void printHelp();
void setMotorSpeed(float speed);
void stopMotors();
void setPIDOutput(float pidOutput);
long getMotor1Position();
long getMotor2Position();
void resetMotorPositions();

void setup() { //stepper_setup
  Serial.begin(115200);
  Serial.println("Stepper Motor Base Code for Self-Balancing Robot");
  
  // Configure enable pins
  pinMode(MOTOR1_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR2_ENABLE_PIN, OUTPUT);
  
  // Enable motors (LOW = enabled for most drivers like A4988/DRV8825)
  digitalWrite(MOTOR1_ENABLE_PIN, LOW);
  digitalWrite(MOTOR2_ENABLE_PIN, LOW);
  
  // Configure Motor 1 (Left)
  motor1.setMaxSpeed(MAX_SPEED);
  motor1.setAcceleration(ACCELERATION);
  motor1.setSpeed(0);
  
  // Configure Motor 2 (Right)
  motor2.setMaxSpeed(MAX_SPEED);
  motor2.setAcceleration(ACCELERATION);
  motor2.setSpeed(0);
  
  Serial.println("Motors initialized. Ready for control.");
  Serial.println("Commands: f=forward, b=backward, s=stop, +/-=speed adj");
}

void loop() {//stepper_loop
  // Handle serial commands for testing
  handleSerialCommands();
  
  // Run the motors
  if (motorsEnabled) {
    runMotors();
  }
  
  // Print status every 1000ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    printStatus();
    lastPrint = millis();
  }
}

void runMotors() {
  // Set motor speeds (negative for motor2 to make robot go forward)
  motor1.setSpeed(motorSpeed);
  motor2.setSpeed(-motorSpeed);  // Reverse for proper forward motion
  
  // Run the motors
  motor1.runSpeed();
  motor2.runSpeed();
}

void setMotorSpeed(float speed) {
  // Constrain speed to maximum limits
  motorSpeed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  
  Serial.print("Motor speed set to: ");
  Serial.println(motorSpeed);
}

void stopMotors() {
  motorSpeed = 0;
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  Serial.println("Motors stopped");
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch (command) {
      case 'f':  // Forward
        setMotorSpeed(300);
        break;
        
      case 'b':  // Backward
        setMotorSpeed(-300);
        break;
        
      case 's':  // Stop
        stopMotors();
        break;
        
      case '+':  // Increase speed
        setMotorSpeed(motorSpeed + 50);
        break;
        
      case '-':  // Decrease speed
        setMotorSpeed(motorSpeed - 50);
        break;
        
      case 'e':  // Enable motors
        motorsEnabled = true;
        digitalWrite(MOTOR1_ENABLE_PIN, LOW);  // LOW = enabled
        digitalWrite(MOTOR2_ENABLE_PIN, LOW);
        Serial.println("Motors enabled");
        break;
        
      case 'd':  // Disable motors
        motorsEnabled = false;
        digitalWrite(MOTOR1_ENABLE_PIN, HIGH); // HIGH = disabled
        digitalWrite(MOTOR2_ENABLE_PIN, HIGH);
        stopMotors();
        Serial.println("Motors disabled");
        break;
        
      case '?':  // Help
        printHelp();
        break;
    }
  }
}

void printStatus() {
  Serial.print("Speed: ");
  Serial.print(motorSpeed);
  Serial.print(" | Enabled: ");
  Serial.print(motorsEnabled ? "YES" : "NO");
  Serial.print(" | Motor1 Pos: ");
  Serial.print(motor1.currentPosition());
  Serial.print(" | Motor2 Pos: ");
  Serial.println(motor2.currentPosition());
}

void printHelp() {
  Serial.println("\n=== STEPPER MOTOR COMMANDS ===");
  Serial.println("f - Move forward");
  Serial.println("b - Move backward");
  Serial.println("s - Stop motors");
  Serial.println("+ - Increase speed");
  Serial.println("- - Decrease speed");
  Serial.println("e - Enable motors");
  Serial.println("d - Disable motors");
  Serial.println("? - Show this help");
  Serial.println("=============================\n");
}

// Function to be called later by PID controller
void setPIDOutput(float pidOutput) {
  float mappedSpeed = constrain(pidOutput, -MAX_SPEED, MAX_SPEED);
  setMotorSpeed(mappedSpeed);
}

// Function to get current motor positions (useful for odometry)
long getMotor1Position() {
  return motor1.currentPosition();
}

long getMotor2Position() {
  return motor2.currentPosition();
}

// Function to reset motor positions
void resetMotorPositions() {
  motor1.setCurrentPosition(0);
  motor2.setCurrentPosition(0);
  Serial.println("Motor positions reset to 0");
}

#ifndef STEPPER_TEST_H
#define STEPPER_TEST_H

#include <AccelStepper.h>

// Function declarations
void stepper_setup();
void stepper_loop();
void runMotors();
void setMotorSpeed(float speed);
void stopMotors();
void handleSerialCommands();
void printStatus();
void printHelp();
void setPIDOutput(float pidOutput);
long getMotor1Position();
long getMotor2Position();
void resetMotorPositions();

#endif

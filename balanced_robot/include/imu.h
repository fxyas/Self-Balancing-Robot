#ifndef IMU_H
#define IMU_H

void initIMU();
void calibrateIMU();
void updateIMU();
float getPitchAngle();
bool isIMUReady();
void resetIMUAngle();

extern float pitch_angle;
extern bool imu_calibrated;

#endif

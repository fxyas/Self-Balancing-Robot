#include <Wire.h>
#include <Arduino.h>
#include "imu.h"

// MPU6050 Configuration
#define MPU6050_ADDR 0x68
#define GYRO_SCALE 131.0        // LSB/°/s for ±250°/s range
#define ACCEL_SCALE 8192.0      // LSB/g for ±4g range (16384 for ±2g)
#define RAD_TO_DEG 57.2958      // 180/π

// Calibration settings
#define CALIBRATION_SAMPLES 200
#define CALIBRATION_DELAY 2     // ms between samples

// Filter settings
#define ALPHA 0.98              // Complementary filter coefficient
#define MAX_PITCH_ANGLE 45.0    // Maximum expected pitch angle

// Global variables
float pitch_angle = 0.0;
bool imu_calibrated = false;

// Private variables - Calibration offsets from standalone calibration
static float accel_x_offset = -260.23;
static float accel_y_offset = 12.42;
static float accel_z_offset = -214.84;
static float gyro_x_offset = -144.09;
static float gyro_y_offset = 48.42;
static float gyro_z_offset = -24.68;
static unsigned long last_update_time = 0;
static bool first_run = true;

void initIMU() {
    
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C
    
    // Wake up MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1
    Wire.write(0x00);  // Wake up
    Wire.endTransmission();
    delay(100);
    
    // Configure gyroscope (±250°/s)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B);  // GYRO_CONFIG
    Wire.write(0x00);  // ±250°/s
    Wire.endTransmission();
    
    // Configure accelerometer (±4g)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C);  // ACCEL_CONFIG
    Wire.write(0x08);  // ±4g
    Wire.endTransmission();
    
    // Configure DLPF (Digital Low Pass Filter)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1A);  // CONFIG
    Wire.write(0x03);  // DLPF ~44Hz, Gyro Output Rate 1kHz
    Wire.endTransmission();
    
    delay(100);
}

void calibrateIMU() {
    Serial.println("Using pre-calculated calibration offsets...");
    
    // Display the calibration values being used
    Serial.println("Calibration offsets:");
    Serial.print("  Accel X: "); Serial.println(accel_x_offset);
    Serial.print("  Accel Y: "); Serial.println(accel_y_offset);
    Serial.print("  Accel Z: "); Serial.println(accel_z_offset);
    Serial.print("  Gyro X:  "); Serial.println(gyro_x_offset);
    Serial.print("  Gyro Y:  "); Serial.println(gyro_y_offset);
    Serial.print("  Gyro Z:  "); Serial.println(gyro_z_offset);
    
    // Set calibration as complete since we're using pre-calculated values
    imu_calibrated = true;
    Serial.println("IMU calibration ready!");
    
    // Optional: Quick validation test (read a few samples)
    Serial.println("Validating calibration...");
    delay(500);
    
    for (int i = 0; i < 3; i++) {
        updateIMU();
        Serial.print("Sample "); Serial.print(i + 1);
        Serial.print(": Pitch = "); Serial.print(pitch_angle, 2); Serial.println("°");
        delay(100);
    }
    
    Serial.println("Calibration validation complete!");
}

void updateIMU() {
    if (!imu_calibrated) return;
    
    // Calculate time delta
    unsigned long current_time = millis();
    float dt;
    
    if (first_run) {
        dt = 0.010;  // Assume 10ms for first run
        first_run = false;
    } else {
        dt = (current_time - last_update_time) / 1000.0;
        dt = constrain(dt, 0.005, 0.050);  // Constrain between 5-50ms
    }
    last_update_time = current_time;
    
    // Read accelerometer X, Y, Z (6 bytes total)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);  // ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);
    
    int16_t accel_x_raw = 0, accel_y_raw = 0, accel_z_raw = 0;
    if (Wire.available() == 6) {
        accel_x_raw = (Wire.read() << 8) | Wire.read();
        accel_y_raw = (Wire.read() << 8) | Wire.read();
        accel_z_raw = (Wire.read() << 8) | Wire.read();
    }
    
    // Read gyroscope Y (pitch rate)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x45);  // GYRO_YOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 2, true);
    
    int16_t gyro_y_raw = 0;
    if (Wire.available() == 2) {
        gyro_y_raw = (Wire.read() << 8) | Wire.read();
    }
    
    // Convert to physical units with calibration offsets applied
    float accel_x_g = ((float)accel_x_raw - accel_x_offset) / ACCEL_SCALE;
    float accel_y_g = ((float)accel_y_raw - accel_y_offset) / ACCEL_SCALE;
    float accel_z_g = ((float)accel_z_raw - accel_z_offset) / ACCEL_SCALE;
    float gyro_y_dps = ((float)gyro_y_raw - gyro_y_offset) / GYRO_SCALE;
    
    // Calculate pitch angle from accelerometer (more accurate method)
    // For X-forward orientation: pitch = atan2(-accel_x, sqrt(accel_y² + accel_z²))
    float accel_magnitude_yz = sqrt(accel_y_g * accel_y_g + accel_z_g * accel_z_g);
    float accel_pitch = atan2(-accel_x_g, accel_magnitude_yz) * RAD_TO_DEG;
    
    // Complementary filter
    // High-pass filter on gyro, low-pass filter on accelerometer
    pitch_angle = ALPHA * (pitch_angle + gyro_y_dps * dt) + (1.0 - ALPHA) * accel_pitch;
    
    // Constrain output to reasonable range
    pitch_angle = constrain(pitch_angle, -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE);
}

float getPitchAngle() {
    return pitch_angle;
}

bool isIMUReady() {
    return imu_calibrated;
}

// Optional: Function to reset IMU angle (useful for manual reset)
void resetIMUAngle() {
    pitch_angle = 0.0;
    first_run = true;
}
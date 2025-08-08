#include <Wire.h>
#include <Arduino.h>
#include "imu.h"

// MPU6050 Configuration
#define MPU6050_ADDR 0x68
#define GYRO_SCALE 131.0        // LSB/°/s for ±250°/s range
#define ACCEL_SCALE 8192.0      // LSB/g for ±4g range
#define RAD_TO_DEG 57.2958      // 180/π

// Calibration settings
#define CALIBRATION_SAMPLES 2000
#define CALIBRATION_DELAY 2     // ms between samples

// Filter settings
#define ALPHA 0.98              // Complementary filter coefficient
#define MAX_PITCH_ANGLE 45.0    // Maximum expected pitch angle

// Global variables
float pitch_angle = 0.0;
bool imu_calibrated = false;

// Private variables
static float gyro_y_offset = 0.0;
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
    Serial.println("Calibrating IMU... Keep robot stationary!");
    
    long gyro_sum = 0;
    int valid_samples = 0;
    
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        // Read gyroscope Y (pitch rate)
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x45);  // GYRO_YOUT_H
        Wire.endTransmission(false);
        Wire.requestFrom(MPU6050_ADDR, 2, true);
        
        if (Wire.available() == 2) {
            int16_t gyro_raw = (Wire.read() << 8) | Wire.read();
            
            // Reject obvious outliers during calibration
            if (abs(gyro_raw) < 1000) {  // Reasonable stationary threshold
                gyro_sum += gyro_raw;
                valid_samples++;
            }
        }
        
        delay(CALIBRATION_DELAY);
        
        // Progress indicator
        if (i % 200 == 0) {
            Serial.print(".");
        }
    }
    
    if (valid_samples > CALIBRATION_SAMPLES * 0.8) {  // At least 80% valid samples
        gyro_y_offset = (float)gyro_sum / valid_samples;
        imu_calibrated = true;
        Serial.println("\nIMU calibration complete!");
        Serial.print("Gyro Y offset: ");
        Serial.println(gyro_y_offset);
    } else {
        Serial.println("\nIMU calibration failed! Too much movement detected.");
        imu_calibrated = false;
    }
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
    
    // Read accelerometer X (forward/backward for pitch)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);  // ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 2, true);
    
    int16_t accel_x_raw = 0;
    if (Wire.available() == 2) {
        accel_x_raw = (Wire.read() << 8) | Wire.read();
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
    
    // Convert to physical units
    float accel_x_g = (float)accel_x_raw / ACCEL_SCALE;
    float gyro_y_dps = ((float)gyro_y_raw - gyro_y_offset) / GYRO_SCALE;
    
    // Calculate pitch angle from accelerometer
    // For X-forward orientation: pitch = atan2(-accel_x, sqrt(accel_y² + accel_z²))
    // Simplified for small angles: pitch ≈ -asin(accel_x/g)
    accel_x_g = constrain(accel_x_g, -1.0, 1.0);  // Prevent asin domain error
    float accel_pitch = -asin(accel_x_g) * RAD_TO_DEG;
    
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
// imu.cpp
#include <Wire.h>
#include <Arduino.h>
#include "imu.h"

// ===== MPU6050 Configuration =====
#define MPU6050_ADDR 0x68
#define GYRO_SCALE 131.0        // LSB/°/s for ±250°/s range
#define ACCEL_SCALE 16384.0     // LSB/g for ±2g range (better for balancing)
#define RAD_TO_DEG 57.2958      // 180/π

// ===== Calibration Settings =====
#define CALIBRATION_SAMPLES 500  // More samples for better calibration
#define CALIBRATION_DELAY 2      // ms between samples

// ===== Filter Settings =====
#define COMPLEMENTARY_ALPHA 0.98 // 98% gyro, 2% accelerometer
#define INITIAL_ANGLE_SAMPLES 50 // Samples for initial angle detection

// ===== Global Variables =====
float pitch_angle = 0.0;
bool imu_calibrated = false;

// ===== Private Variables =====
static float accel_x_offset = 293.52;
static float accel_y_offset = -66.62;
static float accel_z_offset = -199.75;
static float gyro_x_offset = -136.84;
static float gyro_y_offset = 39.57;
static float gyro_z_offset = -22.94;
static unsigned long last_update_time = 0;
static bool first_run = true;

// ===== INITIALIZATION =====
void initIMU() {
    Serial.println("Initializing MPU6050...");
    
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C for faster communication
    
    // Wake up MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0x00);  // Wake up (disable sleep mode)
    Wire.endTransmission();
    delay(100);
    
    // Configure gyroscope (±250°/s for better resolution)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B);  // GYRO_CONFIG register
    Wire.write(0x00);  // ±250°/s
    Wire.endTransmission();
    
    // Configure accelerometer (±2g for better resolution)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C);  // ACCEL_CONFIG register
    Wire.write(0x00);  // ±2g
    Wire.endTransmission();
    
    // Configure Digital Low Pass Filter for stability
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1A);  // CONFIG register
    Wire.write(0x04);  // DLPF ~20Hz for accelerometer, ~20Hz for gyro
    Wire.endTransmission();
    
    // Optional: Set sample rate divider
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x19);  // SMPRT_DIV register
    Wire.write(0x04);  // Sample rate = 200Hz (1kHz / (1 + 4))
    Wire.endTransmission();
    
    delay(100);
    Serial.println("MPU6050 initialized!");
}

// ===== CALIBRATION =====
void calibrateIMU() {
    Serial.println("Starting IMU calibration...");
    Serial.println("Keep robot still and upright!");
    
    long accel_x_sum = 0, accel_y_sum = 0, accel_z_sum = 0;
    long gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
    
    // Collect samples for calibration
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x3B);  // Starting register for accel readings
        Wire.endTransmission(false);
        Wire.requestFrom(MPU6050_ADDR, 14, true);  // Request 14 bytes
        
        if (Wire.available() == 14) {
            int16_t ax = (Wire.read() << 8) | Wire.read();
            int16_t ay = (Wire.read() << 8) | Wire.read();
            int16_t az = (Wire.read() << 8) | Wire.read();
            Wire.read(); Wire.read();  // Skip temperature
            int16_t gx = (Wire.read() << 8) | Wire.read();
            int16_t gy = (Wire.read() << 8) | Wire.read();
            int16_t gz = (Wire.read() << 8) | Wire.read();
            
            accel_x_sum += ax;
            accel_y_sum += ay;
            accel_z_sum += az;
            gyro_x_sum += gx;
            gyro_y_sum += gy;
            gyro_z_sum += gz;
        }
        
        // Show progress
        if (i % 50 == 0) {
            Serial.print(".");
        }
        
        delay(CALIBRATION_DELAY);
    }
    
    // Calculate offsets
    accel_x_offset = (float)accel_x_sum / CALIBRATION_SAMPLES;
    accel_y_offset = (float)accel_y_sum / CALIBRATION_SAMPLES;
    accel_z_offset = (float)accel_z_sum / CALIBRATION_SAMPLES - ACCEL_SCALE; // Subtract 1g
    gyro_x_offset = (float)gyro_x_sum / CALIBRATION_SAMPLES;
    gyro_y_offset = (float)gyro_y_sum / CALIBRATION_SAMPLES;
    gyro_z_offset = (float)gyro_z_sum / CALIBRATION_SAMPLES;
    
    Serial.println("\nCalibration complete!");
    Serial.println("Offsets:");
    Serial.print("  Accel: X="); Serial.print(accel_x_offset, 1);
    Serial.print(" Y="); Serial.print(accel_y_offset, 1);
    Serial.print(" Z="); Serial.println(accel_z_offset, 1);
    Serial.print("  Gyro:  X="); Serial.print(gyro_x_offset, 1);
    Serial.print(" Y="); Serial.print(gyro_y_offset, 1);
    Serial.print(" Z="); Serial.println(gyro_z_offset, 1);
    
    // Set initial angle from accelerometer
    float initial_angle = 0;
    for (int i = 0; i < INITIAL_ANGLE_SAMPLES; i++) {
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU6050_ADDR, 6, true);
        
        if (Wire.available() == 6) {
            int16_t ax = (Wire.read() << 8) | Wire.read();
            int16_t ay = (Wire.read() << 8) | Wire.read();
            int16_t az = (Wire.read() << 8) | Wire.read();
            
            float accel_x_g = ((float)ax - accel_x_offset) / ACCEL_SCALE;
            float accel_z_g = ((float)az - accel_z_offset) / ACCEL_SCALE;
            
            initial_angle += atan2(-accel_x_g, accel_z_g) * RAD_TO_DEG;
        }
        delay(5);
    }
    
    pitch_angle = initial_angle / INITIAL_ANGLE_SAMPLES;
    Serial.print("Initial pitch angle: ");
    Serial.print(pitch_angle, 2);
    Serial.println("°");
    
    imu_calibrated = true;
    last_update_time = millis();
}

// ===== UPDATE READINGS =====
void updateIMU() {
    if (!imu_calibrated) return;
    
    // Calculate time delta
    unsigned long current_time = millis();
    float dt;
    
    if (first_run) {
        dt = 0.010;  // 10ms for first iteration
        first_run = false;
    } else {
        dt = (current_time - last_update_time) / 1000.0;
        // Constrain dt to reasonable values
        if (dt < 0.001 || dt > 0.1) {
            last_update_time = current_time;
            return;
        }
    }
    last_update_time = current_time;
    
    // Read all sensor data at once (more efficient)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);  // Start with ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);  // Read 14 bytes
    
    if (Wire.available() != 14) return;  // Data not ready
    
    // Read raw values
    int16_t accel_x_raw = (Wire.read() << 8) | Wire.read();
    int16_t accel_y_raw = (Wire.read() << 8) | Wire.read();
    int16_t accel_z_raw = (Wire.read() << 8) | Wire.read();
    int16_t temp_raw = (Wire.read() << 8) | Wire.read();  // Temperature (unused)
    int16_t gyro_x_raw = (Wire.read() << 8) | Wire.read();
    int16_t gyro_y_raw = (Wire.read() << 8) | Wire.read();
    int16_t gyro_z_raw = (Wire.read() << 8) | Wire.read();
    
    // Apply calibration offsets and convert to physical units
    float accel_x_g = ((float)accel_x_raw - accel_x_offset) / ACCEL_SCALE;
    float accel_z_g = ((float)accel_z_raw - accel_z_offset) / ACCEL_SCALE;
    float gyro_y_dps = ((float)gyro_y_raw - gyro_y_offset) / GYRO_SCALE;
    
    // Calculate pitch from accelerometer
    // Using atan2 for full range (-90° to +90°)
    float accel_pitch = atan2(-accel_x_g, accel_z_g) * RAD_TO_DEG;
    
    // Apply complementary filter
    // Gyro for short-term accuracy, accelerometer for long-term stability
    pitch_angle = COMPLEMENTARY_ALPHA * (pitch_angle + gyro_y_dps * dt) + 
                  (1.0 - COMPLEMENTARY_ALPHA) * accel_pitch;
    
    // Optional: Apply simple moving average for extra smoothing
    static float angle_buffer[3] = {0, 0, 0};
    static int buffer_index = 0;
    
    angle_buffer[buffer_index] = pitch_angle;
    buffer_index = (buffer_index + 1) % 3;
    
    float smoothed_angle = (angle_buffer[0] + angle_buffer[1] + angle_buffer[2]) / 3.0;
    pitch_angle = smoothed_angle;
}

// ===== GETTERS =====
float getPitchAngle() {
    return pitch_angle;
}

bool isIMUReady() {
    return imu_calibrated;
}

// ===== UTILITIES =====
void resetIMUAngle() {
    pitch_angle = 0.0;
    first_run = true;
    Serial.println("IMU angle reset to 0°");
}

// Optional: Get raw sensor data for debugging
void getRawIMUData(int16_t* accel, int16_t* gyro) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);
    
    if (Wire.available() == 14) {
        accel[0] = (Wire.read() << 8) | Wire.read();
        accel[1] = (Wire.read() << 8) | Wire.read();
        accel[2] = (Wire.read() << 8) | Wire.read();
        Wire.read(); Wire.read();  // Skip temp
        gyro[0] = (Wire.read() << 8) | Wire.read();
        gyro[1] = (Wire.read() << 8) | Wire.read();
        gyro[2] = (Wire.read() << 8) | Wire.read();
    }
}
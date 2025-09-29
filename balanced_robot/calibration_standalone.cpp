#include <Arduino.h>
#include <Wire.h>

// MPU6050 Configuration
#define MPU6050_ADDR 0x68
#define GYRO_SCALE 131.0        // LSB/°/s for ±250°/s range
#define ACCEL_SCALE 8192.0      // LSB/g for ±4g range
#define RAD_TO_DEG 57.2958      // 180/π

// Calibration settings
#define CALIBRATION_SAMPLES 1000  // More samples for better accuracy
#define CALIBRATION_DELAY 5       // ms between samples
#define STABILIZATION_TIME 2000   // ms to wait before starting calibration

// Calibration results
struct CalibrationData {
    float gyro_x_offset;
    float gyro_y_offset;
    float gyro_z_offset;
    float accel_x_offset;
    float accel_y_offset;
    float accel_z_offset;
};

CalibrationData calibration_data;

void initMPU6050() {
    Serial.println("Initializing MPU6050...");
    
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
    
    // Test connection
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x75);  // WHO_AM_I register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 1, true);
    
    if (Wire.available()) {
        uint8_t who_am_i = Wire.read();
        Serial.print("WHO_AM_I: 0x");
        Serial.println(who_am_i, HEX);
        if (who_am_i == 0x68 || who_am_i == 0x70) {
            Serial.println("MPU6050 connection successful!");
        } else {
            Serial.println("MPU6050 connection failed!");
            return;
        }
    }
}

void readRawData(int16_t* accel_data, int16_t* gyro_data) {
    // Read accelerometer and gyroscope data (14 bytes total)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);  // ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);
    
    if (Wire.available() == 14) {
        // Read accelerometer data
        accel_data[0] = (Wire.read() << 8) | Wire.read();  // X
        accel_data[1] = (Wire.read() << 8) | Wire.read();  // Y
        accel_data[2] = (Wire.read() << 8) | Wire.read();  // Z
        
        // Skip temperature data (2 bytes)
        Wire.read();
        Wire.read();
        
        // Read gyroscope data
        gyro_data[0] = (Wire.read() << 8) | Wire.read();   // X
        gyro_data[1] = (Wire.read() << 8) | Wire.read();   // Y
        gyro_data[2] = (Wire.read() << 8) | Wire.read();   // Z
    }
}

void performCalibration() {
    Serial.println("\n=== MPU6050 CALIBRATION ===");
    Serial.println("Place the IMU on a flat, stable surface.");
    Serial.println("Do NOT move the sensor during calibration!");
    Serial.print("Starting in ");
    
    // Countdown
    for (int i = 5; i > 0; i--) {
        Serial.print(i);
        Serial.print("... ");
        delay(1000);
    }
    Serial.println("GO!");
    
    // Wait for stabilization
    Serial.println("Stabilizing...");
    delay(STABILIZATION_TIME);
    
    // Initialize sums
    long accel_sum[3] = {0, 0, 0};
    long gyro_sum[3] = {0, 0, 0};
    int valid_samples = 0;
    
    Serial.println("Collecting calibration data...");
    
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        int16_t accel_raw[3], gyro_raw[3];
        readRawData(accel_raw, gyro_raw);
        
        // Check for reasonable values (reject obvious outliers)
        bool valid_sample = true;
        for (int j = 0; j < 3; j++) {
            if (abs(gyro_raw[j]) > 1000) {  // Gyro should be near zero when stationary
                valid_sample = false;
                break;
            }
        }
        
        if (valid_sample) {
            for (int j = 0; j < 3; j++) {
                accel_sum[j] += accel_raw[j];
                gyro_sum[j] += gyro_raw[j];
            }
            valid_samples++;
        }
        
        // Progress indicator
        if (i % 100 == 0) {
            Serial.print("Progress: ");
            Serial.print((i * 100) / CALIBRATION_SAMPLES);
            Serial.println("%");
        }
        
        delay(CALIBRATION_DELAY);
    }
    
    Serial.print("Valid samples: ");
    Serial.print(valid_samples);
    Serial.print(" / ");
    Serial.println(CALIBRATION_SAMPLES);
    
    if (valid_samples > CALIBRATION_SAMPLES * 0.8) {  // At least 80% valid samples
        // Calculate offsets
        calibration_data.accel_x_offset = (float)accel_sum[0] / valid_samples;
        calibration_data.accel_y_offset = (float)accel_sum[1] / valid_samples;
        calibration_data.accel_z_offset = (float)accel_sum[2] / valid_samples - ACCEL_SCALE; // Remove 1g from Z
        
        calibration_data.gyro_x_offset = (float)gyro_sum[0] / valid_samples;
        calibration_data.gyro_y_offset = (float)gyro_sum[1] / valid_samples;
        calibration_data.gyro_z_offset = (float)gyro_sum[2] / valid_samples;
        
        Serial.println("\n=== CALIBRATION SUCCESSFUL ===");
        printCalibrationResults();
        
    } else {
        Serial.println("\n=== CALIBRATION FAILED ===");
        Serial.println("Too much movement detected during calibration!");
        Serial.println("Please ensure the sensor is completely stationary and try again.");
    }
}

void printCalibrationResults() {
    Serial.println("\n--- Raw Offset Values ---");
    Serial.print("Accel X Offset: "); Serial.println(calibration_data.accel_x_offset, 2);
    Serial.print("Accel Y Offset: "); Serial.println(calibration_data.accel_y_offset, 2);
    Serial.print("Accel Z Offset: "); Serial.println(calibration_data.accel_z_offset, 2);
    Serial.print("Gyro X Offset:  "); Serial.println(calibration_data.gyro_x_offset, 2);
    Serial.print("Gyro Y Offset:  "); Serial.println(calibration_data.gyro_y_offset, 2);
    Serial.print("Gyro Z Offset:  "); Serial.println(calibration_data.gyro_z_offset, 2);
    
    Serial.println("\n--- Code to Copy to imu.cpp ---");
    Serial.println("// Add these offset values to your imu.cpp:");
    Serial.print("static float accel_x_offset = "); Serial.print(calibration_data.accel_x_offset, 2); Serial.println(";");
    Serial.print("static float accel_y_offset = "); Serial.print(calibration_data.accel_y_offset, 2); Serial.println(";");
    Serial.print("static float accel_z_offset = "); Serial.print(calibration_data.accel_z_offset, 2); Serial.println(";");
    Serial.print("static float gyro_x_offset = "); Serial.print(calibration_data.gyro_x_offset, 2); Serial.println(";");
    Serial.print("static float gyro_y_offset = "); Serial.print(calibration_data.gyro_y_offset, 2); Serial.println(";");
    Serial.print("static float gyro_z_offset = "); Serial.print(calibration_data.gyro_z_offset, 2); Serial.println(";");
    
    Serial.println("\n--- Physical Units ---");
    Serial.print("Accel X: "); Serial.print(calibration_data.accel_x_offset / ACCEL_SCALE, 4); Serial.println(" g");
    Serial.print("Accel Y: "); Serial.print(calibration_data.accel_y_offset / ACCEL_SCALE, 4); Serial.println(" g");
    Serial.print("Accel Z: "); Serial.print(calibration_data.accel_z_offset / ACCEL_SCALE, 4); Serial.println(" g");
    Serial.print("Gyro X:  "); Serial.print(calibration_data.gyro_x_offset / GYRO_SCALE, 4); Serial.println(" °/s");
    Serial.print("Gyro Y:  "); Serial.print(calibration_data.gyro_y_offset / GYRO_SCALE, 4); Serial.println(" °/s");
    Serial.print("Gyro Z:  "); Serial.print(calibration_data.gyro_z_offset / GYRO_SCALE, 4); Serial.println(" °/s");
}

void testCalibration() {
    Serial.println("\n=== TESTING CALIBRATION ===");
    Serial.println("Reading 10 samples with calibration applied...");
    
    for (int i = 0; i < 10; i++) {
        int16_t accel_raw[3], gyro_raw[3];
        readRawData(accel_raw, gyro_raw);
        
        // Apply calibration
        float accel_x = ((float)accel_raw[0] - calibration_data.accel_x_offset) / ACCEL_SCALE;
        float accel_y = ((float)accel_raw[1] - calibration_data.accel_y_offset) / ACCEL_SCALE;
        float accel_z = ((float)accel_raw[2] - calibration_data.accel_z_offset) / ACCEL_SCALE;
        
        float gyro_x = ((float)gyro_raw[0] - calibration_data.gyro_x_offset) / GYRO_SCALE;
        float gyro_y = ((float)gyro_raw[1] - calibration_data.gyro_y_offset) / GYRO_SCALE;
        float gyro_z = ((float)gyro_raw[2] - calibration_data.gyro_z_offset) / GYRO_SCALE;
        
        Serial.print("Sample "); Serial.print(i + 1); Serial.print(": ");
        Serial.print("Accel("); Serial.print(accel_x, 3); Serial.print(", ");
        Serial.print(accel_y, 3); Serial.print(", "); Serial.print(accel_z, 3); Serial.print(") ");
        Serial.print("Gyro("); Serial.print(gyro_x, 3); Serial.print(", ");
        Serial.print(gyro_y, 3); Serial.print(", "); Serial.print(gyro_z, 3); Serial.println(")");
        
        delay(500);
    }
    
    Serial.println("Calibration test complete!");
    Serial.println("Gyro values should be close to 0.0");
    Serial.println("Accel Z should be close to 1.0, X and Y close to 0.0");
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("\n=== MPU6050 STANDALONE CALIBRATION ===");
    Serial.println("This program will calibrate your MPU6050 sensor.");
    
    initMPU6050();
    delay(1000);
    
    performCalibration();
    
    if (calibration_data.gyro_y_offset != 0) {  // Check if calibration was successful
        testCalibration();
        
        Serial.println("\n=== INSTRUCTIONS ===");
        Serial.println("1. Copy the offset values shown above to your imu.cpp file");
        Serial.println("2. Use these values in your calibrateIMU() function");
        Serial.println("3. Your self-balancing robot should now have better accuracy!");
    }
    
    Serial.println("\nCalibration program complete.");
}

void loop() {
    // Show live sensor readings every 2 seconds
    static unsigned long last_reading = 0;
    if (millis() - last_reading > 2000) {
        int16_t accel_raw[3], gyro_raw[3];
        readRawData(accel_raw, gyro_raw);
        
        Serial.println("\n--- Live Readings ---");
        Serial.print("Raw Accel: "); 
        Serial.print(accel_raw[0]); Serial.print(", ");
        Serial.print(accel_raw[1]); Serial.print(", ");
        Serial.println(accel_raw[2]);
        
        Serial.print("Raw Gyro:  ");
        Serial.print(gyro_raw[0]); Serial.print(", ");
        Serial.print(gyro_raw[1]); Serial.print(", ");
        Serial.println(gyro_raw[2]);
        
        if (calibration_data.gyro_y_offset != 0) {
            float pitch = atan2(-((float)accel_raw[0] - calibration_data.accel_x_offset) / ACCEL_SCALE, 
                               sqrt(pow(((float)accel_raw[1] - calibration_data.accel_y_offset) / ACCEL_SCALE, 2) + 
                                   pow(((float)accel_raw[2] - calibration_data.accel_z_offset) / ACCEL_SCALE, 2))) * RAD_TO_DEG;
            Serial.print("Pitch Angle: "); Serial.print(pitch, 2); Serial.println("°");
        }
        
        last_reading = millis();
    }
}

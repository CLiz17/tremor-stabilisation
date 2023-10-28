#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Offset values obtained during calibration
int offsetAccX = 0;
int offsetAccY = 0;
int offsetAccZ = 0;
int offsetGyroX = 0;
int offsetGyroY = 0;
int offsetGyroZ = 0;

void setup()
{
    Wire.begin();
    Serial.begin(9600);

    mpu.initialize();
    // Apply the offsets obtained during calibration
    mpu.setXAccelOffset(offsetAccX);
    mpu.setYAccelOffset(offsetAccY);
    mpu.setZAccelOffset(offsetAccZ);
}

void loop()
{
    // Read raw accelerometer and gyroscope data
    int16_t accX_raw, accY_raw, accZ_raw;
    int16_t gyroX_raw, gyroY_raw, gyroZ_raw;

    mpu.getMotion6(&accX_raw, &accY_raw, &accZ_raw, &gyroX_raw, &gyroY_raw, &gyroZ_raw);

    // Apply the calibration and sensitivity scale factor conversion to accelerometer data
    float accX = (float)(accX_raw - offsetAccX) / 16384.0; // Assuming ±2g range
    float accY = (float)(accY_raw - offsetAccY) / 16384.0; // Assuming ±2g range
    float accZ = (float)(accZ_raw - offsetAccZ) / 16384.0; // Assuming ±2g range

    // Print the calibrated accelerometer data
    Serial.print("Calibrated AccX: ");
    Serial.print(accX);
    Serial.print(", Calibrated AccY: ");
    Serial.print(accY);
    Serial.print(", Calibrated AccZ: ");
    Serial.print(accZ);
    Serial.println();

    delay(10); // in millisec
}
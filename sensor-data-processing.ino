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

    // Calculate pitch and roll angles from accelerometer data
    float calculatedPitch = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180.0 / PI;
    float calculatedRoll = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;

    // Print the calculated pitch and roll angles
    Serial.print("Calculated Pitch: ");
    Serial.print(calculatedPitch);
    Serial.print("°, Calculated Roll: ");
    Serial.print(calculatedRoll);
    Serial.println("°");

    delay(10); // in millisec
}
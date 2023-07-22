#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo servoMotor;

// Offset values obtained during calibration
int offsetAccX = 0;
int offsetAccY = 0;
int offsetAccZ = 0;
int offsetGyroX = 0;
int offsetGyroY = 0;
int offsetGyroZ = 0;

// Variables to store filtered orientation angles
float filteredPitch = 0.0;
float filteredRoll = 0.0;

// Tuning parameter for the Complementary Filter
float alpha = 0.98; // 0 < alpha < 1

const float dt = 0.01; // Time interval between each iteration (in seconds)

// Variables to store previous gyroscope data
float previous_gyroX = 0.0;
float previous_gyroY = 0.0;

// Pin for the servo motor
const int servoPin = 9; // Change this to your desired PWM pin

// Map angles to servo range
const int servoMinAngle = 0;
const int servoMaxAngle = 180;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  mpu.initialize();
  // Apply the offsets obtained during calibration
  mpu.setXAccelOffset(offsetAccX);
  mpu.setYAccelOffset(offsetAccY);
  mpu.setZAccelOffset(offsetAccZ);

  // Attach the servo to the PWM pin
  servoMotor.attach(servoPin);
}

void loop() {
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

  // Integrate gyroscope data to update pitch and roll angles
  float gyroX_rate = (gyroX_raw - offsetGyroX) / 131.0; // Assuming ±250°/s range
  float gyroY_rate = (gyroY_raw - offsetGyroY) / 131.0; // Assuming ±250°/s range

  filteredPitch = alpha * (filteredPitch + (gyroX_rate + previous_gyroX) * 0.5 * dt) + (1 - alpha) * calculatedPitch;
  filteredRoll = alpha * (filteredRoll + (gyroY_rate + previous_gyroY) * 0.5 * dt) + (1 - alpha) * calculatedRoll;

  // Save current gyroscope data for the next iteration
  previous_gyroX = gyroX_rate;
  previous_gyroY = gyroY_rate;

  // Map the filtered angles to the servo range
  int mappedPitch = map(filteredPitch, -90, 90, servoMinAngle, servoMaxAngle);
  int mappedRoll = map(filteredRoll, -90, 90, servoMinAngle, servoMaxAngle);

  // Apply constraints
  mappedPitch = constrain(mappedPitch, servoMinAngle, servoMaxAngle);
  mappedRoll = constrain(mappedRoll, servoMinAngle, servoMaxAngle);

  // Update the servo position based on the filtered angles
  servoMotor.write(mappedPitch);

  // Print the filtered pitch and roll angles
  Serial.print("Filtered Pitch: ");
  Serial.print(filteredPitch);
  Serial.print("°, Filtered Roll: ");
  Serial.print(filteredRoll);
  Serial.println("°");

  delay(10); // in millisec
}
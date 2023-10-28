# Arduino Sensor Data Processing and Servo Control

This repository contains three Arduino sketches for processing sensor data and controlling a servo motor. The codes are designed for interfacing with an MPU6050 sensor to perform orientation estimation and use the estimated angles to control a servo motor.

## Repository Contents

1. **Orientation_Estimation_With_Servo**:

   - File: `orientation-estimation-with-servo.ino`
   - Description: This code reads sensor data from an MPU6050 sensor, performs orientation estimation using a complementary filter, and controls a servo motor based on the estimated angles. It combines accelerometer and gyroscope data to estimate pitch and roll angles and adjusts the servo motor's position accordingly.

2. **MPU6050_Data_Processing**:

   - File: `sensor-data-processing.ino`
   - Description: This code reads sensor data from an MPU6050 sensor, applies calibration and sensitivity scale factor conversion to accelerometer and gyroscope data, calculates pitch and roll angles, and prints the results to the serial monitor. It's a fundamental script for processing data from the MPU6050 sensor.

3. **MPU6050_Data_Calibration**:
   - File: `sensor-data-calibration.ino`
   - Description: This code is used for calibrating an MPU6050 sensor. It captures raw accelerometer and gyroscope data and allows users to obtain offset values for calibration. These offset values can then be used in other scripts to improve data accuracy.

## Getting Started

To use these codes, follow these steps:

1. **Hardware Setup**:

   - Connect your MPU6050 sensor and servo motor to your Arduino Nano board following the wiring instructions.
   - Ensure that the necessary libraries are installed, including `Wire.h`, `MPU6050.h`, and `Servo.h`.

2. **Code Upload**:

   - Upload the desired Arduino sketch to your Arduino board using the Arduino IDE or a compatible development environment.

3. **Serial Monitor**:

   - Open the serial monitor to observe the output data and any debugging information.

4. **Adjustments**:
   - Depending on your specific setup and requirements, you may need to adjust the code parameters, tuning constants, or pin assignments.

## Contributions

Contributions to this repository are welcome. You can contribute by:

- Reporting issues or bugs.
- Suggesting improvements or enhancements.
- Adding new features or functionalities.
- Providing documentation updates.

Please create a pull request to contribute to this repository.

Happy coding!

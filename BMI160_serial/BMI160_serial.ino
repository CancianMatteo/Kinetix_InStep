#include <Wire.h>
 
// I2C Configuration for ESP32
#define BMI160_I2C_ADDRESS 0x69  // I2C address for BMI160 (with SAO pin → disconnected or 3.3V), connect to GND for 0x68
#define ACCEL_SENSITIVITY 16384.0 // Sensitivity for ±2g in LSB/g (adjust based on your configuration)
 
void setup() {
  Serial.begin(115200); // Initialize Serial communication
  Wire.begin();         // Initialize I2C communication
 
  // Initialize BMI160 accelerometer
  Wire.beginTransmission(BMI160_I2C_ADDRESS);
  Wire.write(0x7E); // Command register
  Wire.write(0x11); // Set accelerometer to normal mode
  Wire.endTransmission();
  delay(100);
 
  // Perform accelerometer auto-calibration
  autoCalibrateAccelerometer();
 
  Serial.println("BMI160 Initialized and Calibrated");
}
 
void loop() {
  int16_t ax, ay, az;
 
  // Read accelerometer data
  Wire.beginTransmission(BMI160_I2C_ADDRESS);
  Wire.write(0x12); // Start register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(BMI160_I2C_ADDRESS, 6);
 
  if (Wire.available() == 6) {
    ax = (Wire.read() | (Wire.read() << 8));
    ay = (Wire.read() | (Wire.read() << 8)); 
    az = (Wire.read() | (Wire.read() << 8));
  }
 
  // Convert raw accelerometer values to m/s^2
  float ax_mps2 = ax /ACCEL_SENSITIVITY *9.81;
  float ay_mps2 = ay /ACCEL_SENSITIVITY *9.81;
  float az_mps2 = az /ACCEL_SENSITIVITY *9.81;
 
  // Print accelerometer values in m/s^2
  Serial.print("Acceleration (m/s^2): ");
  Serial.print(ax_mps2-0.1, 2);
  Serial.print(", ");
  Serial.print(ay_mps2+0.7, 2);
  Serial.print(", ");
  Serial.print(az_mps2, 2);
  Serial.print(". \t");

  // Convert raw accelerometer values to g
  float ax_g = ax / ACCEL_SENSITIVITY;
  float ay_g = ay / ACCEL_SENSITIVITY;
  float az_g = az / ACCEL_SENSITIVITY;
 
  // Calculate tilt angles (pitch and roll) in degrees
  float pitch = atan2(ay_g, sqrt(ax_g * ax_g + az_g * az_g)) * 180.0 / PI;
  float roll = atan2(-ax_g, az_g) * 180.0 / PI;
 
  // Print tilt angles
  Serial.print("Pitch: ");
  Serial.print(pitch+4.5, 2);
  Serial.print("°, Roll: ");
  Serial.print(roll+178.6, 2);
  Serial.println("°");
 
  delay(500);
}

/**
 * @brief Set the accelerometer and gyroscope to normal mode with a range of ±2g and ±2000°/s respectively
 */
/*void setBMI160NormalMode() {
  BMI160.setAccelerometerRange(BMI160GenClass::ACCEL_RANGE_2);
  BMI160.setAccelerometerRate(BMI160GenClass::ACCEL_BW_NORMAL);
  BMI160.setGyroRange(BMI160GenClass::GYRO_RANGE_2000DPS);
  BMI160.setGyroRate(BMI160GenClass::GYRO_BW_NORMAL);
}*/

void autoCalibrateAccelerometer() {
  // Configure accelerometer for auto-calibration
  Wire.beginTransmission(BMI160_I2C_ADDRESS);
  Wire.write(0x7E); // Command register
  Wire.write(0x37); // Start accelerometer offset calibration
  Wire.endTransmission();
 
  // Wait for calibration to complete
  delay(1000);
  Serial.println("Accelerometer Auto-Calibration Complete");
}
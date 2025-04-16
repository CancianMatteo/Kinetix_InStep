#include <BMI160Gen.h>            //https://github.com/hanyazou/BMI160-Arduino
#include <Wire.h>
 
// I2C Configuration for ESP32
#define i2c_addr 0x68  // I2C address for BMI160 (with SAO pin → GND), connect to 3V3 for 0x69
#define BMI160_sda_pin 22     // I2C D4(GPIO22) → SDA Pin for XIAO ESP32C6
#define BMI160_scl_pin 23     // I2C D5(GPIO23) → SCL Pin for XIAO ESP32C6

int ax, ay, az; // Raw accelerometer values
int offset_ax=1070, offset_ay=-160, offset_az=16480; // Offset values for accelerometer
int gx, gy, gz; // Raw gyroscope values
int offset_gx=-25, offset_gy=-65, offset_gz=-40; // Offset values for gyroscope
 
void setup() {
  // Initialize Serial communication at 115200 baud rate
  Serial.begin(115200);
 
  // Initialize I2C with custom SDA and SCL pins
  Wire.begin(sda_pin, scl_pin);
 
  // Initialize the BMI160 device in I2C mode
  if (!BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr)) {
    Serial.println("BMI160 initialization failed!");
    while (1); // Halt if initialization fails
  }
 
  Serial.println("BMI160 initialized successfully in I2C mode!");
  delay(1000);

  // Set the accelerometer and gyroscope to normal mode with a range of ±2g and ±2000°/s respectively
  BMI160.setAccelerometerNormalMode(BMI160GenClass::ACCEL_RANGE_2G, BMI160GenClass::ACCEL_BW_NORMAL);
}

void loop() {
  // Read raw accelerometer measurements from the BMI160
  BMI160.readAccelerometer(ay, ax, az);  //TOCHANGE based on orientation
 
  // Read raw gyroscope measurements from the BMI160
  BMI160.readGyro(gy, gx, gz);  //TOCHANGE based on orientation
 
  // Display the accelerometer values (X, Y, Z) on the Serial Monitor
  Serial.print("Acc: ");
  Serial.print(ax+offset_ax);
  Serial.print(",  ");
  Serial.print(ay+offset_ay);
  Serial.print(",  ");
  Serial.print(az+offset_az);
 
  // Display the gyroscope values (X, Y, Z) on the Serial Monitor
  Serial.print("\tGyro: ");
  Serial.print(gx+offset_gx);
  Serial.print(",  ");
  Serial.print(gy+offset_gy);
  Serial.print(",  ");
  Serial.println(gz+offset_gz);

  // Wait for 500ms before the next reading
  delay(500);
}
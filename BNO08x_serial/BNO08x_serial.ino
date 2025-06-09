/***************************************************************************
* Example sketch for the SparkFun BNO08x library
* 
* This sketch shows how to retrieve accelerometer, gyroscope, and 
* magnetometer data from the BNO08x using I2C.
* 
* Library: https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library
***************************************************************************/

#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

#define BNO08X_I2C_ADDR 0x4B // Default I2C address for BNO08x (AD0/SA0 low). Use 0x4B if high.
#define SAMPLE_TIME 1000

BNO08x myIMU;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  delay(500);

  while (!myIMU.begin(BNO08X_I2C_ADDR, Wire)) {
    Serial.println("BNO08x not detected. Check wiring!");
    delay(100);
  }
  Serial.println("BNO08x connected!");

  // Enable reports for accelerometer, gyroscope, and magnetometer
  setReports();

  Serial.println("Reading events");
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (myIMU.enableAccelerometer(SAMPLE_TIME) != true) {
    Serial.println("Could not enable accelerometer");
  }
  if (myIMU.enableGyro(SAMPLE_TIME) != true) {
    Serial.println("Could not enable gyro");
  }
  if (myIMU.enableMagnetometer(SAMPLE_TIME) != true) {
    Serial.println("Could not enable magnetometer");
  }
}

void loop() {
  if (myIMU.wasReset()) {
    Serial.print("sensor was reset :");
    Serial.print(myIMU.getResetReason());
    setReports();
  }

  if (myIMU.getSensorEvent()) {
    // Accelerometer
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER) {
      Serial.print("Accel (m/s^2): ");
      Serial.print(myIMU.getAccelX(), 2); Serial.print("\t");
      Serial.print(myIMU.getAccelY(), 2); Serial.print("\t");
      Serial.print(myIMU.getAccelZ(), 2); Serial.print("\t");
    }
    byte accuracy = myIMU.getAccelAccuracy();
    switch(accuracy){
      case 0: Serial.println("Unreliable"); break;
      case 1: Serial.println("Low"); break;
      case 2: Serial.println("Medium"); break;
      case 3: Serial.println("High"); break;
    }

    // Gyroscope
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
      Serial.print("Gyro (rad/s): ");
      Serial.print(myIMU.getGyroX(), 2); Serial.print("\t");
      Serial.print(myIMU.getGyroY(), 2); Serial.print("\t");
      Serial.print(myIMU.getGyroZ(), 2); Serial.print("\t");
    }
    accuracy = myIMU.getGyroAccuracy();
    switch(accuracy){
      case 0: Serial.println("Unreliable"); break;
      case 1: Serial.println("Low"); break;
      case 2: Serial.println("Medium"); break;
      case 3: Serial.println("High"); break;
    }

    // Magnetometer
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_MAGNETIC_FIELD) {
      Serial.print("Mag (uT): ");
      Serial.print(myIMU.getMagX(), 2); Serial.print("\t");
      Serial.print(myIMU.getMagY(), 2); Serial.print("\t");
      Serial.print(myIMU.getMagZ(), 2); Serial.print("\t");
    }
    accuracy = myIMU.getMagAccuracy();   // from 0=Unreliable to 3=High
    switch(accuracy){
      case 0: Serial.println("Unreliable"); break;
      case 1: Serial.println("Low"); break;
      case 2: Serial.println("Medium"); break;
      case 3: Serial.println("High"); break;
    }
  }

  delay(500); // ~2Hz output rate
}
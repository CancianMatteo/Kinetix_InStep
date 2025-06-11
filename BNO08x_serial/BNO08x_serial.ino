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
#define SAMPLE_TIME 100

BNO08x myIMU;
const int REPORT_INTERVAL = SAMPLE_TIME/10; 
unsigned long lastSampleTime;

// For the most reliable interaction with the SHTP bus, we need to use hardware reset control, and to monitor the H_INT pin.
// The H_INT pin will go low when its okay to talk on the SHTP bus.
#define BNO08X_INT  1
#define BNO08X_RST  0

float ax, ay, az, gx, gy, gz, mx, my, mz;
float aAccuracy, gAccuracy, mAccuracy;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  delay(100);

  while (!myIMU.begin(BNO08X_I2C_ADDR, Wire, BNO08X_INT, BNO08X_RST)) {
    Serial.println("BNO08x not detected. Check wiring!");
    delay(100);
  }
  Serial.println("BNO08x connected!");

  // Enable reports for accelerometer, gyroscope, and magnetometer
  setReports();
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports...");
  delay(100);

  while (!myIMU.enableAccelerometer(REPORT_INTERVAL)) {
    Serial.println("Could not enable accelerometer");
    delay(100);
  }
  while (!myIMU.enableGyro(REPORT_INTERVAL)) {
    Serial.println("Could not enable gyro");
    delay(100);
  }
  while (!myIMU.enableMagnetometer(REPORT_INTERVAL)) {
    Serial.println("Could not enable magnetometer");
    delay(100);
  }

  boolean accelInitialized = false;
  boolean gyroInitialized = false;
  boolean magInitialized = false;
  lastSampleTime = millis(); // Initialize the sample timer
  Serial.println("Waiting for initial sensor data...");
  
  // Loop for first accelerometer, gyroscope and magnetometer data to initialize the variables
  while (!(accelInitialized && gyroInitialized && magInitialized)) {
    if (myIMU.wasReset()) {
      Serial.println("Sensor reset during initialization - retrying");
      setReports();
      delay(100);
    }
    
    if (myIMU.getSensorEvent()) {
      switch (myIMU.getSensorEventID()) {
        case SENSOR_REPORTID_ACCELEROMETER:
          ax = myIMU.getAccelX();
          ay = myIMU.getAccelY();
          az = myIMU.getAccelZ();
          aAccuracy = myIMU.getAccelAccuracy();
          accelInitialized = true;
          break;
          
        case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
          gx = myIMU.getGyroX();
          gy = myIMU.getGyroY();
          gz = myIMU.getGyroZ();
          gAccuracy = myIMU.getGyroAccuracy();
          gyroInitialized = true;
          break;
          
        case SENSOR_REPORTID_MAGNETIC_FIELD:
          mx = myIMU.getMagX();
          my = myIMU.getMagY();
          mz = myIMU.getMagZ();
          mAccuracy = myIMU.getMagAccuracy();
          magInitialized = true;
          break;
      }
    }
    delay(10);
  }
  
  Serial.println("All sensors initialized, starting main loop");
}

void loop() {
  if (myIMU.wasReset()) {
    Serial.print("sensor was reset :");
    Serial.println(myIMU.getResetReason());
    setReports();
  }

  if (myIMU.getSensorEvent()) {
    // Accelerometer
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER) {
      ax = (ax*4+myIMU.getAccelX())/5.0;
      ay = (ay*4+myIMU.getAccelY())/5.0;
      az = (az*4+myIMU.getAccelZ())/5.0;
      aAccuracy = (aAccuracy*4+myIMU.getAccelAccuracy())/5.0;
    }

    // Gyroscope
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
      gx = (gx*4+myIMU.getGyroX())/5.0;
      gy = (gy*4+myIMU.getGyroY())/5.0;
      gz = (gz*4+myIMU.getGyroZ())/5.0;
      gAccuracy = (gAccuracy*4+myIMU.getGyroAccuracy()/5.0;
    }

    // Magnetometer
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_MAGNETIC_FIELD) {
      mx = (mx*4+myIMU.getMagX())/5.0;
      my = (my*4+myIMU.getMagY())/5.0;
      mz = (mz*4+myIMU.getMagZ())/5.0;
      mAccuracy = (mAccuracy*4+myIMU.getMagAccuracy())/5.0;
    }
  }

  // Print IMU data every SAMPLE_TIME ms
  if (millis() - lastSampleTime >= SAMPLE_TIME) {
    Serial.print("Accel: ");  // m/s^2
    Serial.print(ax, 2); Serial.print("\t");
    Serial.print(ay, 2); Serial.print("\t");
    Serial.print(az, 2); Serial.print("\t");
    Serial.print(getAccuracyString(aAccuracy));
    Serial.print("\t Gyro: ");   // rad/s
    Serial.print(gx, 2); Serial.print("\t");
    Serial.print(gy, 2); Serial.print("\t");
    Serial.print(gz, 2); Serial.print("\t");
    Serial.print(getAccuracyString(gAccuracy));
    Serial.print("\t Mag: ");    // uT
    Serial.print(mx, 2); Serial.print("\t");
    Serial.print(my, 2); Serial.print("\t");
    Serial.print(mz, 2); Serial.print("\t");
    Serial.println(getAccuracyString(mAccuracy));

    lastSampleTime = millis();
  }

  delayMicroseconds(10);
}

// Function to print accuracy as string
const char* getAccuracyString(float accuracy) {
  if(accuracy<0.5) {
    return "Bad";
  }else if(accuracy<1.5) {
    return "Low";
  }else if(accuracy<2.5) {
    return "Good";
  }else {
    return "High";
  }
}

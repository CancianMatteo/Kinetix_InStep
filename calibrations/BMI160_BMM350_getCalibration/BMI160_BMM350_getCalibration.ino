#include <DFRobot_BMM350.h>
#include <DFRobot_BMI160.h>

DFRobot_BMM350_I2C bmm350(&Wire, I2C_ADDRESS);
DFRobot_BMI160 bmi160;

#define BMI160_I2C_ADDRESS_1 0x68 
#define BMI160_I2C_ADDRESS_2 0x69

void setup() 
{
  Serial.begin(115200);
  while (!Serial);


  Serial.println("DFRobot BMM350 & BMI160 getCalibration");
  while (bmm350.begin()) {
    Serial.println("bmm350 init failed, Please try again!");
    delay(500);
  }
  Serial.println("bmm350 init success!");

  /**
   * Set sensor operation mode
   * opMode:
   *   eBmm350SuspendMode      // suspend mode: Suspend mode is the default power mode of BMM350 after the chip is powered, Current consumption in suspend mode is minimal, 
   *                               so, this mode is useful for periods when data conversion is not needed. Read and write of all registers is possible.
   *   eBmm350NormalMode       // normal mode  Get geomagnetic data normally.
   *   eBmm350ForcedMode       // forced mode  Single measurement, the sensor restores to suspend mode when the measurement is done.
   *   eBmm350ForcedModeFast  // To reach ODR = 200Hz is only possible by using FM_ FAST.
   */
  bmm350.setOperationMode(eBmm350NormalMode);
  /**
   * Set preset mode, make it easier for users to configure sensor to get geomagnetic data (The default rate for obtaining geomagnetic data is 12.5Hz)
   * presetMode:
   *   BMM350_PRESETMODE_LOWPOWER      // Low power mode, get a fraction of data and take the mean value.
   *   BMM350_PRESETMODE_REGULAR       // Regular mode, get a number of data and take the mean value.
   *   BMM350_PRESETMODE_ENHANCED      // Enhanced mode, get a plenty of data and take the mean value.
   *   BMM350_PRESETMODE_HIGHACCURACY  // High accuracy mode, get a huge number of take and draw the mean value.
   * rate:
   *   BMM350_DATA_RATE_1_5625HZ
   *   BMM350_DATA_RATE_3_125HZ
   *   BMM350_DATA_RATE_6_25HZ
   *   BMM350_DATA_RATE_12_5HZ   (default rate)
   *   BMM350_DATA_RATE_25HZ
   *   BMM350_DATA_RATE_50HZ
   *   BMM350_DATA_RATE_100HZ
   *   BMM350_DATA_RATE_200HZ
   *   BMM350_DATA_RATE_400HZ
   */
  bmm350.setPresetMode(BMM350_PRESETMODE_HIGHACCURACY,BMM350_DATA_RATE_50HZ);
  /**
   * Enable the measurement at x-axis, y-axis and z-axis, default to be enabled, no config required, the geomagnetic data at x, y and z will be inaccurate when disabled.
   * Refer to setMeasurementXYZ() function in the .h file if you want to configure more parameters.
   */
  bmm350.setMeasurementXYZ();


  // detect, set and init the bmi160 (soft reset included)
  while (bmi160.I2cInit(BMI160_I2C_ADDRESS_1)!=BMI160_OK && bmi160.I2cInit(BMI160_I2C_ADDRESS_2)!=BMI160_OK){
    Serial.println("init false");
    delay(500);
  }

  //set the bmi160 range and sample rate
  //bmi160.setSensConf(BMI160_ACCEL_SEL, BMI160_ACCEL_RANGE_16G, BMI160_ACCEL_ODR_50HZ);
  //bmi160.setSensConf(BMI160_GYRO_SEL, BMI160_GYRO_RANGE_1000_DPS, BMI160_GYRO_ODR_50HZ);
  /*DFRobot_BMI160::sBmm160SensorData_t accelConf;
  accelConf.range = DFRobot_BMI160::eAccelRange_16G;
  accelConf.odr = DFRobot_BMI160::eAccelODR_50Hz;
  bmi160.setAccelConf(accelConf);

  DFRobot_BMI160::sBmm160SensorData_t gyroConf;
  gyroConf.range = DFRobot_BMI160::eGyroRange_1000DPS;
  gyroConf.odr = DFRobot_BMI160::eGyroODR_50Hz;
  bmi160.setGyroConf(gyroConf);*/
}

int last_Read=0;
int16_t accelGyro[6]={0}; 

void loop() 
{
  while (millis()-last_Read<20);
  last_Read = millis();

  //get both accel and gyro data from bmi160 into the pointed array
  if (bmi160.getAccelGyroData(accelGyro) != 0){
    for (int i=0; i<6; i++){
      accelGyro[i] = 0; 
    }
  }

  sBmm350MagData_t magData = bmm350.getGeomagneticData();
    Serial.print("Raw:");
    Serial.print(accelGyro[3]);
    Serial.print(',');
    Serial.print(accelGyro[4]);
    Serial.print(',');
    Serial.print(accelGyro[5]);
    Serial.print(',');
    Serial.print(accelGyro[0]);
    Serial.print(',');
    Serial.print(accelGyro[1]);
    Serial.print(',');
    Serial.print(accelGyro[2]);
    Serial.print(',');
    Serial.print(magData.x*10);
    Serial.print(',');
    Serial.print(magData.y*10);
    Serial.print(',');
    Serial.print(magData.z*10);
    Serial.println();
  
  delay(10);
}
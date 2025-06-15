// === Libraries ===
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <time.h>
#include "BMM350.h"
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <ICM20948_WE.h>
#include <TinyGPSPlus.h>

// === WiFi === (smartphone's hotspot)
const char* ssid = "PC-MATTEO";
const char* password = "matteooo";

// === MQTT === (broker Mosquitto on Mac Air)
const char* mqtt_server = "192.168.137.212";  // broker's IP address
String mqtt_topic = "calcio/prova";    // topic to publish data
#define MQTT_MAX_PACKET_SIZE 60000 // Increase max packet size to handle larger payloads
WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);
String payload = ""; // Payload to be published via MQTT
unsigned int nPublish = 0;

// === Configuration ===
bool configReceived = false;
String efuseMacStr = String((uint64_t)ESP.getEfuseMac(), HEX);
String player, exercise_type;

// === Time ===
const char* ntpServer1 = "it.pool.ntp.org";
const char* ntpServer2 = "ntp1.inrim.it";
const long gmtOffset_sec = 3600;      // UTC+1 for Italy/Venice (CET)
const int daylightOffset_sec = 3600;  // +1 hour for summer time (CEST), so in total +2 hours            
String fromTime;                      // When current buffer started collecting
unsigned long lastNTPSync = millis();

#define MCU "ESP32"
#define BUILTIN_USER_LED 21     // GPIO21 = LED giallo USER

// ===== IMUs config =====
#define IMU_SAMPLE_RATE 10     // advised not to exceed 100Hz
#define PUBLISH_INTERVAL 5      // [in seconds] publish every n second
#define RANGE_ACC 16             // Accelerometer range ±16G
#define RANGE_GYR 2000          // Gyroscope range ±2000dps
// IMU_SAMPLE_RATE * PUBLISH_INTERVAL ≤ 100 to avoid negative impact on performance (example 20Hz * 5s = 100 samples 👍)
// going above 100 samples will most probably cause issues and delays with MQTT publish due to large payload size

// First IMU: BMI160 + BMM350
#define BMI160_I2C_ADDRESS 0x69
#define BMM350_I2C_ADDRESS 0x14
BMM350 imuBMM(BMM350_I2C_ADDRESS); // or 0x15
float hardIron_BMM350[3] = {0, 0, 0};
float softIron_BMM350[3][3] = {{1,0,0},{0,1,0},{0,0,1}};

// Second IMU: BNO08X
#define BNO08X_I2C_ADDR 0x4B // Default I2C address for BNO08x (AD0 disconnected or ->3V3). Use 0x4A if connected to GND.
#define BNO08X_INT  1  // Adjust pin numbers to match your connections
#define BNO08X_RST  0
BNO08x imuBNO;
const int REPORT_INTERVAL = (1000/IMU_SAMPLE_RATE)/5;
int accuracyA;
int accuracyG;
int accuracyM;
// BNO08X does iron calibration automatically, so no need to set hard/soft iron

// Third IMU: ICM20948
#define ICM20948_ADDR 0x68  // 0x69 is I2C address for ICM-20948 (with AD0 pin → disconnected or 3.3V), connect to GND for 0x68
ICM20948_WE imuICM = ICM20948_WE(ICM20948_ADDR);
float hardIron_ICM20948[3] = {0, 0, 0};
float softIron_ICM20948[3][3] = {{1,0,0},{0,1,0},{0,0,1}};

float aRes, gRes;
unsigned long nextIMUSampleTime;

// ===== GPS config =====
#define GPS_RX_PIN 21     // TX(GPIO21) → RX
#define GPS_TX_PIN 20     // RX(GPIO20) → TX
#define GPS_SAMPLE_RATE 1    // GPS data sample rate

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // Use UART1 on ESP32
bool hasGPSFix = false;
unsigned long nextGPSSampleTime;

// === Structs & Buffers ===
struct IMUData {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
};

struct GPSData {
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t centisecond;
  double lat;
  double lng;
  float spd;
};

IMUData avgDataBNO; // Struct to hold temp BNO08x data

IMUData imuBuffer[IMU_SAMPLE_RATE*PUBLISH_INTERVAL];
int imuIndex = 0;
GPSData gpsBuffer[GPS_SAMPLE_RATE*PUBLISH_INTERVAL];
int gpsIndex = 0;

// === Function Prototypes ===
void readIMUs(IMUData *imuData);
void readBMIandBMM(IMUData *BMIBMMData);
void handleReportBNO08X(int &accuracyA, int &accuracyG, int &accuracyM);
void readICM20948(IMUData *ICMData);
void readGPS(GPSData *gpsData);
void setReportsBNO08X();
void writeByteI2C(TwoWire &wire, uint8_t address, uint8_t subAddress, uint8_t data);
void readBytesI2C(TwoWire &wire, uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest);
void config_ICM_20948();

// === Setup ===
void setup() {
  pinMode(BUILTIN_USER_LED, OUTPUT);
  digitalWrite(BUILTIN_USER_LED, LOW); // Turn on the LED while configuring (active low)
  Serial.begin(115200);
  Wire.begin();

  aRes = (float)RANGE_ACC / 32768.f;  // Accelerometer resolution
  gRes = (float)RANGE_GYR / 32768.f; // Gyroscope resolution

  // BMI160 + BMM350
  BMI160_begin();
  while (!imuBMM.begin(&Wire)) {
    Serial.println("Failed to initialize BMM350! Check your wiring.");
    delay(500);
  }
  imuBMM.setRateAndPerformance(BMM350_DATA_RATE_100HZ, BMM350_ULTRALOWNOISE);
  Serial.println("BMI160 and BMM350 are connected!");

  // BNO08X
  while (!imuBNO.begin(BNO08X_I2C_ADDR, Wire, BNO08X_INT, BNO08X_RST)) {
    Serial.println("BNO08x not detected. Check wiring!");
    delay(100);
  }
  setReportsBNO08X();
  Serial.println("BNO08x is connected!");

  // ICM20948
  while(!imuICM.init()){
    Serial.println("ICM20948 does not respond");
    delay(100);
  }
  while(!imuICM.initMagnetometer()){
    Serial.println("Magnetometer does not respond");
    delay(500);
  }
  Serial.println("ICM20948 is connected!");
  config_ICM_20948();

  // WiFi
  Serial.print("WiFi is setting up..");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.print("\nWiFi connected, IP: ");
  Serial.println(WiFi.localIP());

  Serial.print("Free heap: ");
  Serial.println(ESP.getFreeHeap());

  // MQTT
  mqtt_client.setBufferSize(MQTT_MAX_PACKET_SIZE);
  Serial.print("MQTT buffer size: ");
  Serial.println(mqtt_client.getBufferSize());
  mqtt_client.setServer(mqtt_server, 1883);
  mqtt_client.setKeepAlive(300);
  connectMQTT();

  waitForConfig();
  
  configTimeWithNTP();
  
  // GPS check fix (wait up to triesLeft seconds)
  gpsSerial.begin(115200, SERIAL_8N1, GPS_TX_PIN, GPS_RX_PIN);
  Serial.println("\nGPS is setting up...");

  for (int triesLeft = 30; !hasGPSFix && triesLeft>0; triesLeft--) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }
    if (gps.satellites.isValid()) {
      Serial.print("📡 Sat: ");
      Serial.print(gps.satellites.value());
    } else {
      Serial.print("📡 Sat: --");
    }
    Serial.println(" | ❌ No Fix Yet");
    delay(1000);
  }
  if (gps.location.isValid() && gps.satellites.value()>=3) {
    hasGPSFix = true;
    Serial.print("📡 Sat: ");
    Serial.print(gps.satellites.value());
    Serial.print(" | ✅ Fix: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", ");
    Serial.println(gps.location.lng(), 6);
  }

  digitalWrite(BUILTIN_USER_LED, HIGH); // Turn off the LED after configuration
  delay(1000); // Give some time before starting the loop

  Serial.println("Setup complete. Ready to read IMU and GPS data.");
  nextIMUSampleTime = millis();
  nextGPSSampleTime = millis();
}

// === Loop ===
void loop() {
  // Time to sample IMU?
  if (millis() >= nextIMUSampleTime) {
    // Check if we missed more than one sample period
    int missedSamples = (millis() - nextIMUSampleTime) / (1000 / IMU_SAMPLE_RATE);
    if (missedSamples > 0)
      Serial.printf("Warning⚠️: Missed %d IMU samples\n", missedSamples);
    
    // Read IMUs data
    if (imuIndex < IMU_SAMPLE_RATE * PUBLISH_INTERVAL) {
      readIMUs(&imuBuffer[imuIndex]);    
      imuIndex++;
      // Schedule next sample time
      nextIMUSampleTime += (1000 / IMU_SAMPLE_RATE) * (missedSamples+1);
    }
  }else {
    // Read BNO08X reports continuously to ensure data is fresh
    handleReportBNO08X(accuracyA, accuracyG, accuracyM);
  }

  // Time to sample GPS?
  if (hasGPSFix) {
    if (millis() >= nextGPSSampleTime) {
      // Check if we missed more than one sample period
      int missedSamples = (millis() - nextGPSSampleTime) / (1000 / GPS_SAMPLE_RATE);
      if (missedSamples > 0)
        Serial.printf("Warning⚠️: Missed %d GPS samples\n", missedSamples);
      
      // Read GPS data
      if (gpsIndex < GPS_SAMPLE_RATE * PUBLISH_INTERVAL) {
        readGPS(&gpsBuffer[gpsIndex]);
        gpsIndex++;
        // Schedule next sample time
        nextGPSSampleTime += (1000 / GPS_SAMPLE_RATE) * (missedSamples+1);
      }
    }
  } else {
    // If GPS fix is not available, try to read GPS data
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }
    if (gps.location.isValid() && gps.satellites.value() >= 3) {
      hasGPSFix = true;
      Serial.print("📡 Sat: ");
      Serial.print(gps.satellites.value());
      Serial.print(" | ✅ Fix: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(", ");
      Serial.println(gps.location.lng(), 6);
    } else {
      Serial.println("📡 No valid GPS data yet");
    }
  }

  // Record the start time when we begin collecting a new buffer
  if (imuIndex == 1) {
    time_t bufferStartTime;  
    time(&bufferStartTime);
    fromTime = getISOTimestamp(bufferStartTime);

    // Resync NTP time once per hour to prevent drift
    if (millis() - lastNTPSync > 3600000) { // 1 hour
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
      lastNTPSync = millis();
    }
  }

  // Publish when buffer is full
  if (imuIndex==IMU_SAMPLE_RATE*PUBLISH_INTERVAL && gpsIndex==GPS_SAMPLE_RATE*PUBLISH_INTERVAL) {
    digitalWrite(BUILTIN_USER_LED, LOW); // Turn on LED (active low)
    
    unsigned long buildStartTime = millis();
    payload = buildMQTTPayload();
    
    unsigned long publishStartTime = millis();
    if (payload.length() > 0) {
      if (publishData(payload)) {
        Serial.printf("%d° message published (%lums, %lums)\n", 
                      ++nPublish, publishStartTime - buildStartTime, millis() - publishStartTime);
      }
    } else {
      Serial.println("Failed to build payload");
    }
    
    imuIndex = 0;
    gpsIndex = 0;
    payload = "";
    digitalWrite(BUILTIN_USER_LED, HIGH); // Turn off LED
  }

  delayMicroseconds(100);
}

// === Start BMI160 === 
void BMI160_begin() {
  writeByteI2C(Wire, BMI160_I2C_ADDRESS, 0x7E, 0xB6);  // Soft reset
  delay(100);
  writeByteI2C(Wire, BMI160_I2C_ADDRESS, 0x7E, 0x11);  // Accelerometer normal mode
  delay(100);
  writeByteI2C(Wire, BMI160_I2C_ADDRESS, 0x7E, 0x15);  // Gyroscope normal mode
  delay(100);
  writeByteI2C(Wire, BMI160_I2C_ADDRESS, 0x40, 0x28);  // Accelerometer ODR = 100Hz
  writeByteI2C(Wire, BMI160_I2C_ADDRESS, 0x42, 0x28);  // Gyroscope ODR = 100Hz
  writeByteI2C(Wire, BMI160_I2C_ADDRESS, 0x41, 0x0C);  // RANGE_ACC => ±16G
  writeByteI2C(Wire, BMI160_I2C_ADDRESS, 0x43, 0x00);  // RANGE_GYR => ±2000dps
}

void autoCalibrateBMI160() {
    writeByteI2C(Wire, BMI160_I2C_ADDRESS, 0x69, 0x6F); // Write 0b01101111 (0b0gxxyyzz, see BMI160 datasheet pg. 78)
    writeByteI2C(Wire, BMI160_I2C_ADDRESS, 0x7E, 0x37); // Start accelerometer offset calibration
    writeByteI2C(Wire, BMI160_I2C_ADDRESS, 0x7E, 0x03); // Trigger auto-calibration (issuing start_foc command)
    // Wait for calibration to complete
    delay(500);
}

// Here is where you define the sensor outputs you want to receive
void setReportsBNO08X() {
  Serial.println("Setting desired reports...");
  delay(500);

  while (!imuBNO.enableAccelerometer(REPORT_INTERVAL)) {
    Serial.println("Could not enable accelerometer");
    delay(100);
  }
  while (!imuBNO.enableGyro(REPORT_INTERVAL)) {
    Serial.println("Could not enable gyro");
    delay(100);
  }
  while (!imuBNO.enableMagnetometer(REPORT_INTERVAL)) {
    Serial.println("Could not enable magnetometer");
    delay(100);
  }

  boolean accelInitialized = false;
  boolean gyroInitialized = false;
  boolean magInitialized = false;
  
  // Loop for first accelerometer, gyroscope and magnetometer data to initialize the variables
  while (!(accelInitialized && gyroInitialized && magInitialized)) {
    if (imuBNO.wasReset()) {
      Serial.println("Sensor reset during initialization - retrying");
      setReportsBNO08X();
      delay(100);
    }

    if (imuBNO.getSensorEvent()) {
      switch (imuBNO.getSensorEventID()) {
        case SENSOR_REPORTID_ACCELEROMETER:
          avgDataBNO.ax = imuBNO.getAccelX()/9.81; // Convert m/s² to g
          avgDataBNO.ay = imuBNO.getAccelY()/9.81;
          avgDataBNO.az = imuBNO.getAccelZ()/9.81;
          accuracyA = imuBNO.getAccelAccuracy();
          accelInitialized = true;
          break;
          
        case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
          avgDataBNO.gx = imuBNO.getGyroX()*180.0/PI; // Convert rad/s to deg/s
          avgDataBNO.gy = imuBNO.getGyroY()*180.0/PI;
          avgDataBNO.gz = imuBNO.getGyroZ()*180.0/PI;
          accuracyG = imuBNO.getGyroAccuracy();
          gyroInitialized = true;
          break;
          
        case SENSOR_REPORTID_MAGNETIC_FIELD:
          avgDataBNO.mx = imuBNO.getMagX();
          avgDataBNO.my = imuBNO.getMagY();
          avgDataBNO.mz = imuBNO.getMagZ();
          accuracyM = imuBNO.getMagAccuracy();
          magInitialized = true;
          break;
      }
    }
    delay(10);
  }
}

void config_ICM_20948(){
  /******************* Calibration ******************/ 
  
  /*  This is a method to calibrate. You have to determine the minimum and maximum 
   *  raw acceleration values of the axes determined in the range +/- 2 g. 
   *  You call the function as follows: setAccOffsets(xMin,xMax,yMin,yMax,zMin,zMax);
   *  The calibration changes the slope / ratio of raw acceleration vs g. The zero point is set as (min + max)/2.
   */
  imuICM.setAccOffsets(-16330.0, 16450.0, -16600.0, 16180.0, -16520.0, 16690.0);
    
  /* The starting point, if you position the ICM20948 flat, is not necessarily 0g/0g/1g for x/y/z. 
   * The autoOffset function measures offset. It assumes your ICM20948 is positioned flat with its x,y-plane. The more you deviate from this, the less accurate will be your results.
   * It overwrites the zero points of setAccOffsets, but keeps the correction of the slope.
   * The function also measures the offset of the gyroscope data. The gyroscope offset does not depend on the positioning.
   * This function needs to be called after setAccOffsets but before other settings since it will overwrite settings!
   * You can query the offsets with the functions:xyzFloat getAccOffsets() and xyzFloat getGyrOffsets()
   * You can apply the offsets using: setAccOffsets(xyzFloat yourOffsets) and setGyrOffsets(xyzFloat yourOffsets)
   */
  Serial.println("Position your ICM20948 flat with the chip looking upward and don't move it - calibrating...");
  delay(500);
  imuICM.autoOffsets();
  delay(500);
  Serial.println("Done!"); 
  
  /*  The gyroscope data is not zero, even if you don't move the ICM20948. 
   *  To start at zero, you can apply offset values. These are the gyroscope raw values you obtain
   *  using the +/- 250 degrees/s range. 
   *  Use either autoOffset or setGyrOffsets, not both.
   */
  //imuICM.setGyrOffsets(-115.0, 130.0, 105.0);
  
  /*  ICM20948_ACC_RANGE_2G      2 g   (default)
   *  ICM20948_ACC_RANGE_4G      4 g
   *  ICM20948_ACC_RANGE_8G      8 g   
   *  ICM20948_ACC_RANGE_16G    16 g
   */
  imuICM.setAccRange(ICM20948_ACC_RANGE_16G);
  
  /*  Choose a level for the Digital Low Pass Filter or switch it off.  
   *  ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7, ICM20948_DLPF_OFF 
   *    ASRD = Accelerometer Sample Rate Divider (0...4095)
   *    You achieve lowest noise using level 6 
   *  IMPORTANT: This needs to be ICM20948_DLPF_7 if DLPF is used in cycle mode!
   *  
   *  DLPF       3dB Bandwidth [Hz]      Output Rate [Hz]
   *    0              246.0               1125/(1+ASRD) 
   *    1              246.0               1125/(1+ASRD)
   *    2              111.4               1125/(1+ASRD)
   *    3               50.4               1125/(1+ASRD)
   *    4               23.9               1125/(1+ASRD)
   *    5               11.5               1125/(1+ASRD)
   *    6                5.7               1125/(1+ASRD) 
   *    7              473.0               1125/(1+ASRD)
   *    OFF           1209.0               4500 
   */
  //imuICM.setAccDLPF(ICM20948_DLPF_OFF);    
  
  /*  Acceleration sample rate divider divides the output rate of the accelerometer.
   *  Sample rate = Basic sample rate / (1 + divider) 
   *  It can only be applied if the corresponding DLPF is not off!
   *  Divider is a number 0...4095 (different range compared to gyroscope)
   *  If sample rates are set for the accelerometer and the gyroscope, the gyroscope
   *  sample rate has priority.
   */
  //imuICM.setAccSampleRateDivider(100);
  
  /*  ICM20948_GYRO_RANGE_250       250 degrees per second (default)
   *  ICM20948_GYRO_RANGE_500       500 degrees per second
   *  ICM20948_GYRO_RANGE_1000     1000 degrees per second
   *  ICM20948_GYRO_RANGE_2000     2000 degrees per second
   */
  imuICM.setGyrRange(ICM20948_GYRO_RANGE_2000);
  
  /*  Choose a level for the Digital Low Pass Filter or switch it off. 
   *  ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7, ICM20948_DLPF_OFF 
   *    GSRD = Gyroscope Sample Rate Divider (0...255)
   *    You achieve lowest noise using level 6  
   *  
   *  DLPF       3dB Bandwidth [Hz]      Output Rate [Hz]
   *    0              196.6               1125/(1+GSRD) 
   *    1              151.8               1125/(1+GSRD)
   *    2              119.5               1125/(1+GSRD)
   *    3               51.2               1125/(1+GSRD)
   *    4               23.9               1125/(1+GSRD)
   *    5               11.6               1125/(1+GSRD)
   *    6                5.7               1125/(1+GSRD) 
   *    7              361.4               1125/(1+GSRD)
   *    OFF          12106.0               9000
   */
  //imuICM.setGyrDLPF(ICM20948_DLPF_OFF);  
  
  /*  Gyroscope sample rate divider divides the output rate of the gyroscope.
   *  Sample rate = Basic sample rate / (1 + divider) 
   *  It can only be applied if the corresponding DLPF is not OFF!
   *  Divider is a number 0...255
   *  If sample rates are set for the accelerometer and the gyroscope, the gyroscope
   *  sample rate has priority.
   */
  //imuICM.setGyrSampleRateDivider(100);

  /*  Choose a level for the Digital Low Pass Filter. 
   *  ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7, ICM20948_DLPF_OFF 
   *    You achieve lowest noise using level 6  
   *  
   *  DLPF          Bandwidth [Hz]      Output Rate [Hz]
   *    0             7932.0                    9
   *    1              217.9                 1125
   *    2              123.5                 1125
   *    3               65.9                 1125
   *    4               34.1                 1125
   *    5               17.3                 1125
   *    6                8.8                 1125
   *    7             7932.0                    9
   */
  //imuICM.setTempDLPF(ICM20948_DLPF_OFF);
 
  /* You can set the following modes for the magnetometer:
   * AK09916_PWR_DOWN          Power down to save energy
   * AK09916_TRIGGER_MODE      Measurements on request, a measurement is triggered by 
   *                           calling setMagOpMode(AK09916_TRIGGER_MODE)
   * AK09916_CONT_MODE_10HZ    Continuous measurements, 10 Hz rate
   * AK09916_CONT_MODE_20HZ    Continuous measurements, 20 Hz rate
   * AK09916_CONT_MODE_50HZ    Continuous measurements, 50 Hz rate
   * AK09916_CONT_MODE_100HZ   Continuous measurements, 100 Hz rate (default)
   */
  imuICM.setMagOpMode(AK09916_CONT_MODE_100HZ);
  delay(100); // add a delay of at least 1000/magRate to avoid first mag value being zero 
}

// === Get Config via MQTT ===
void callback(char* topic, byte* conf_payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);
  if (String(topic) == "init/config") {
    StaticJsonDocument<1024> doc;
    DeserializationError err = deserializeJson(doc, conf_payload, length);
    if (err) {
      Serial.println("JSON parse failed");
      return;
    }
    if (doc.containsKey(efuseMacStr)) {
      JsonObject obj = doc[efuseMacStr];
      if (obj.containsKey("mqtt_topic")) mqtt_topic = obj["mqtt_topic"].as<String>();
      if (obj.containsKey("player")) player = obj["player"].as<String>();
      if (obj.containsKey("exercise_type")) exercise_type = obj["exercise_type"].as<String>();
      if (obj.containsKey("imu_calibration")){
        JsonObject imuCalib = obj["imu_calibration"];
        if (imuCalib.containsKey("hard_iron_BMM350")) {
          for (int i = 0; i < 3; i++)
            hardIron_BMM350[i] = imuCalib["hard_iron_BMM350"][i];
        }
        if (imuCalib.containsKey("soft_iron_BMM350")) {
          for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
              softIron_BMM350[i][j] = imuCalib["soft_iron_BMM350"][i][j];
        }
        if (imuCalib.containsKey("hard_iron_ICM20948")) {
          for (int i = 0; i < 3; i++)
            hardIron_ICM20948[i] = imuCalib["hard_iron_ICM20948"][i];
        }
        if (imuCalib.containsKey("soft_iron_ICM20948")) {
          for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
              softIron_ICM20948[i][j] = imuCalib["soft_iron_ICM20948"][i][j];
        }
      }
      if (obj.containsKey("playground")) {
        JsonObject playground = obj["playground"];
        if (playground.containsKey("lat_low_left")) {
          float lat_low_left = playground["lat_low_left"];
          float long_low_left = playground["long_low_left"];
          float lat_high_right = playground["lat_high_right"];
          float long_high_right = playground["long_high_right"];
          Serial.printf("Playground coordinates: [%f, %f] to [%f, %f]\n", lat_low_left, long_low_left, lat_high_right, long_high_right);
        }
      }

      // Apply calibrations to IMUs
      imuBMM.setCalibration(hardIron_BMM350, softIron_BMM350);

      configReceived = true;
      Serial.println("Config received and applied.");
    }
  }
}

void waitForConfig() {
  mqtt_client.setCallback(callback);
  mqtt_client.subscribe("init/config");
  Serial.print(efuseMacStr);
  Serial.println(" waiting for config on topic init/config...");

  unsigned long startTime = millis();
  while (!configReceived) {
    mqtt_client.loop();
    delay(100);
    if (millis() - startTime > 100000) { // Timeout after 100 seconds
      Serial.println("Config not received, proceeding with default values.");
      break;
    }
  }
  mqtt_client.unsubscribe("init/config");
  mqtt_client.setCallback(NULL); // Remove callback if not needed anymore
}

// === Config Time ===
void configTimeWithNTP() {
  // Configure time with NTP servers
  Serial.println("Configuring time with NTP...");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
  
  // Wait for time to be set (max 10 seconds)
  unsigned long startAttemptTime = millis();
  time_t now = 0;
  Serial.print("Waiting for NTP time sync");
  while (time(&now) < 1600000000 && millis() - startAttemptTime < 10000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  
  // Display current time
  if (now > 1600000000) {
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    Serial.print("Current time: ");
    Serial.print(timeinfo.tm_year + 1900);
    Serial.print("-");
    Serial.print(timeinfo.tm_mon + 1);
    Serial.print("-");
    Serial.print(timeinfo.tm_mday);
    Serial.print(" ");
    Serial.print(timeinfo.tm_hour);
    Serial.print(":");
    Serial.print(timeinfo.tm_min);
    Serial.print(":");
    Serial.println(timeinfo.tm_sec);
  } else {
    Serial.println("Failed to get time from NTP, will use epoch time");
  }
}

// === Read IMUs Data ===
void readIMUs(IMUData *imuData) {
  IMUData BMIBMMData = {0};
  IMUData ICMData = {0};

  // Read data from each IMU
  readBMIandBMM(&BMIBMMData);
  // Note: BNO08X data is continuously collected via handleReportBNO08X
  handleReportBNO08X(accuracyA, accuracyG, accuracyM);
  readICM20948(&ICMData);

  // Calculate weight for the average of IMUs
  float wBMIa=0.4, wBMIg=0.4, wBMMm=0.5, wBNOa, wBNOg, wBNOm, wICMa, wICMg, wICMm;
  wBNOa = accuracyA /30.0 *4.0;  // scale from 0 to 0.4 based on accuracy
  wBNOg = accuracyG /30.0 *4.0;  // scale from 0 to 0.4 based on accuracy
  wBNOm = accuracyM /30.0 *4.0;  // scale from 0 to 0.4 based on accuracy
  wICMa = 1.0 - (wBMIa + wBNOa);
  wICMg = 1.0 - (wBMIg + wBNOg);
  wICMm = 1.0 - (wBMMm + wBNOm);

  Serial.println("\nAccel X: " +String(BMIBMMData.ax, 2)+", \t" +String(avgDataBNO.ax, 2)+", \t" +String(ICMData.ax, 2)+" g,\n"+
                  "\tY: " +String(BMIBMMData.ay, 2)+", \t" +String(avgDataBNO.ay, 2)+", \t" +String(ICMData.ay, 2)+" g,\n"+
                  "\tZ: " +String(BMIBMMData.az, 2)+", \t" +String(avgDataBNO.az, 2)+", \t" +String(ICMData.az, 2)+" g");
  Serial.println("Gyro X: " +String(BMIBMMData.gx, 2)+", \t" +String(avgDataBNO.gx, 2)+", \t" +String(ICMData.gx, 2)+" °/s,\n"+
                  "\tY: " +String(BMIBMMData.gy, 2)+", \t" +String(avgDataBNO.gy, 2)+", \t" +String(ICMData.gy, 2)+" °/s,\n"+
                  "\tZ: " +String(BMIBMMData.gz, 2)+", \t" +String(avgDataBNO.gz, 2)+", \t" +String(ICMData.gz, 2)+" °/s");
  Serial.println("Mag X: " +String(BMIBMMData.mx, 2)+", \t" +String(avgDataBNO.mx, 2)+", \t" +String(ICMData.mx, 2)+" uT,\n"+
                  "\tY: " +String(BMIBMMData.my, 2)+", \t" +String(avgDataBNO.my, 2)+", \t" +String(ICMData.my, 2)+" uT,\n"+
                  "\tZ: " +String(-BMIBMMData.mz, 2)+", \t" +String(avgDataBNO.mz, 2)+", \t" +String(ICMData.mz, 2)+" uT");

  // Calculate weighted average of accelerometer[G], gyroscope[°/s] and magnetometer data[uT]
  imuData->ax = (BMIBMMData.ax * wBMIa +avgDataBNO.ax * wBNOa +ICMData.ax * wICMa);
  imuData->ay = (BMIBMMData.ay * wBMIa +avgDataBNO.ay * wBNOa +ICMData.ay * wICMa);
  imuData->az = (BMIBMMData.az * wBMIa +avgDataBNO.az * wBNOa +ICMData.az * wICMa);
  imuData->gx = (BMIBMMData.gx * wBMIg +avgDataBNO.gx * wBNOg +ICMData.gx * wICMg);
  imuData->gy = (BMIBMMData.gy * wBMIg +avgDataBNO.gy * wBNOg +ICMData.gy * wICMg);
  imuData->gz = (BMIBMMData.gz * wBMIg +avgDataBNO.gz * wBNOg +ICMData.gz * wICMg);
  imuData->mx = (BMIBMMData.mx * wBMMm +avgDataBNO.mx * wBNOm +ICMData.mx * wICMm);
  imuData->my = (BMIBMMData.my * wBMMm +avgDataBNO.my * wBNOm +ICMData.my * wICMm);
  imuData->mz = (-BMIBMMData.mz * wBMMm +avgDataBNO.mz * wBNOm +ICMData.mz * wICMm); // BMM350 z-axis is inverted (BMM350 upside down)
}

void readBMIandBMM(IMUData *BMIBMMData) {
  // BMI160 accelerometer and gyro data (using register reads)
  int16_t IMUCount[6];
  uint8_t rawData[12];
  readBytesI2C(Wire, BMI160_I2C_ADDRESS, 0x0C, 12, &rawData[0]);
  IMUCount[0] = ((int16_t)rawData[1] << 8) | rawData[0];
  IMUCount[1] = ((int16_t)rawData[3] << 8) | rawData[2];
  IMUCount[2] = ((int16_t)rawData[5] << 8) | rawData[4];
  IMUCount[3] = ((int16_t)rawData[7] << 8) | rawData[6];
  IMUCount[4] = ((int16_t)rawData[9] << 8) | rawData[8];
  IMUCount[5] = ((int16_t)rawData[11] << 8) | rawData[10];

  BMIBMMData->ax = (float)IMUCount[3] * aRes;
  BMIBMMData->ay = (float)IMUCount[4] * aRes;
  BMIBMMData->az = (float)IMUCount[5] * aRes;

  BMIBMMData->gx = (float)IMUCount[0] * gRes;
  BMIBMMData->gy = (float)IMUCount[1] * gRes;
  BMIBMMData->gz = (float)IMUCount[2] * gRes;

  // BMM350 magnetometer data
  imuBMM.readMagnetometerData(BMIBMMData->mx, BMIBMMData->my, BMIBMMData->mz);
}

void handleReportBNO08X(int &accuracyA, int &accuracyG, int &accuracyM){
  if (imuBNO.wasReset()) {
    Serial.print("BNO08X was reset :");
    Serial.println(imuBNO.getResetReason());
    setReportsBNO08X();
  }

  if (imuBNO.getSensorEvent()) {
    // Accelerometer
    if (imuBNO.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER) {
      avgDataBNO.ax = (avgDataBNO.ax*4+imuBNO.getAccelX()/9.81)/5.0;  // Convert m/s² to g
      avgDataBNO.ay = (avgDataBNO.ay*4+imuBNO.getAccelY()/9.81)/5.0;
      avgDataBNO.az = (avgDataBNO.az*4+imuBNO.getAccelZ()/9.81)/5.0;
      accuracyA = (accuracyA*4+imuBNO.getAccelAccuracy())/5.0;
    }

    // Gyroscope
    if (imuBNO.getSensorEventID() == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
      avgDataBNO.gx = (avgDataBNO.gx*4+imuBNO.getGyroX()*180.0/PI)/5.0; // Convert rad/s to deg/s
      avgDataBNO.gy = (avgDataBNO.gy*4+imuBNO.getGyroY()*180.0/PI)/5.0;
      avgDataBNO.gz = (avgDataBNO.gz*4+imuBNO.getGyroZ()*180.0/PI)/5.0;
      accuracyG = (accuracyG*4+imuBNO.getGyroAccuracy())/5.0;
    }

    // Magnetometer
    if (imuBNO.getSensorEventID() == SENSOR_REPORTID_MAGNETIC_FIELD) {
      avgDataBNO.mx = (avgDataBNO.mx*4+imuBNO.getMagX())/5.0;
      avgDataBNO.my = (avgDataBNO.my*4+imuBNO.getMagY())/5.0;
      avgDataBNO.mz = (avgDataBNO.mz*4+imuBNO.getMagZ())/5.0;
      accuracyM = (accuracyM*4+imuBNO.getMagAccuracy())/5.0;
    }
  }
}

void readICM20948(IMUData *ICMData) {
  xyzFloat gValue;
  xyzFloat degValue;
  xyzFloat magValue;
  
  imuICM.readSensor();
  imuICM.getGValues(&gValue);
  imuICM.getGyrValues(&degValue);
  imuICM.getMagValues(&magValue);

  ICMData->ax = gValue.x;
  ICMData->ay = gValue.y;
  ICMData->az = gValue.z;
  ICMData->gx = degValue.x;
  ICMData->gy = degValue.y;
  ICMData->gz = degValue.z;
  ICMData->mx = magValue.x;
  ICMData->my = magValue.y;
  ICMData->mz = magValue.z;
}

// === Read GPS Data ===
void readGPS(GPSData *gpsData) {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated() && gps.satellites.value() >= 3) {
      gpsData->day = gps.date.day();
      gpsData->hour = gps.time.hour();
      gpsData->minute = gps.time.minute();
      gpsData->second = gps.time.second();
      gpsData->centisecond = gps.time.centisecond();
      gpsData->lat = gps.location.lat();
      gpsData->lng = gps.location.lng();
      gpsData->spd = gps.speed.mps();
      return;  // Exit after updating GPS data
    }else{
      Serial.println("GPS data not updated or not enough satellites for a fix");
      delayMicroseconds(100);
    }
  }
}

// === Build Message and Publish Data with MQTT ===
String buildMQTTPayload() {
  if (ESP.getFreeHeap() < MQTT_MAX_PACKET_SIZE+1000) {
    Serial.println("Free heap memory is low, not publishing data");
    return "";
  }

  int estimatedSize = 500 + (imuIndex * 9 * 6); // 9 values * 6 chars avg per value
  String pld;
  pld.reserve(estimatedSize);
  pld = "{\"player\":\"" + player + "\", ";
  pld += "\"source\":\"" + String(MCU) + "-" + efuseMacStr + "\", ";
  pld += "\"exercise_type\":\"" + exercise_type + "\", ";
  pld += "\"fromTime\":\"" + fromTime + "\", ";
  pld += "\"imu\": {";

  // Add arrays for each IMU field
  pld += "\"ax\":[";
  for (int i = 0; i < imuIndex; i++) {
    pld += String(imuBuffer[i].ax, 2);
    if (i < imuIndex - 1) pld += ",";
  }
  pld += "],";

  pld += "\"ay\":[";
  for (int i = 0; i < imuIndex; i++) {
    pld += String(imuBuffer[i].ay, 2);
    if (i < imuIndex - 1) pld += ",";
  }
  pld += "],";

  pld += "\"az\":[";
  for (int i = 0; i < imuIndex; i++) {
    pld += String(imuBuffer[i].az, 2);
    if (i < imuIndex - 1) pld += ",";
  }
  pld += "],";

  pld += "\"gx\":[";
  for (int i = 0; i < imuIndex; i++) {
    pld += String(imuBuffer[i].gx, 2);
    if (i < imuIndex - 1) pld += ",";
  }
  pld += "],";

  pld += "\"gy\":[";
  for (int i = 0; i < imuIndex; i++) {
    pld += String(imuBuffer[i].gy, 2);
    if (i < imuIndex - 1) pld += ",";
  }
  pld += "],";

  pld += "\"gz\":[";
  for (int i = 0; i < imuIndex; i++) {
    pld += String(imuBuffer[i].gz, 2);
    if (i < imuIndex - 1) pld += ",";
  }
  pld += "],";

  pld += "\"mx\":[";
  for (int i = 0; i < imuIndex; i++) {
    pld += String(imuBuffer[i].mx, 2);
    if (i < imuIndex - 1) pld += ",";
  }
  pld += "],";

  pld += "\"my\":[";
  for (int i = 0; i < imuIndex; i++) {
    pld += String(imuBuffer[i].my, 2);
    if (i < imuIndex - 1) pld += ",";
  }
  pld += "],";

  pld += "\"mz\":[";
  for (int i = 0; i < imuIndex; i++) {
    pld += String(imuBuffer[i].mz, 2);
    if (i < imuIndex - 1) pld += ",";
  }
  pld += "]}";

  // Add GPS data if available
  if (hasGPSFix && gpsIndex > 0) {
    pld += ",\"gps\": {";
    pld += "\"time\":[";
    for (int i = 0; i < gpsIndex; i++) {
      pld += String("\"T") + String(gpsBuffer[i].hour) + String("-") + 
             String(gpsBuffer[i].minute) + String("-") +
             String(gpsBuffer[i].second) + String(".") +
             String(gpsBuffer[i].centisecond) + String("\"");
      if (i < gpsIndex - 1) pld += ",";
    }
    pld += "],";

    pld += "\"lat\":[";
    for (int i = 0; i < gpsIndex; i++) {
      pld += String(gpsBuffer[i].lat, 6);
      if (i < gpsIndex - 1) pld += ",";
    }
    pld += "],";

    pld += "\"lng\":[";
    for (int i = 0; i < gpsIndex; i++) {
      pld += String(gpsBuffer[i].lng, 6);
      if (i < gpsIndex - 1) pld += ",";
    }
    pld += "],";

    pld += "\"spd\":[";
    for (int i = 0; i < gpsIndex; i++) {
      pld += String(gpsBuffer[i].spd, 2);
      if (i < gpsIndex - 1) pld += ",";
    }
    pld += "]}";
  }

  return pld+'}';
}

bool publishData(const String& localPayload) {
  if (!mqtt_client.connected()) {
    connectMQTT();
  }

  if (!mqtt_client.beginPublish(mqtt_topic.c_str(), localPayload.length(), true)) {
    Serial.println("beginPublish failed");
    return false;
  }
  
  mqtt_client.print(localPayload);
  
  if (!mqtt_client.endPublish()) {
    Serial.println("endPublish failed");
    return false;
  }
  
  return true;
}

// === Other Functions ===
void connectMQTT() {
  Serial.print("Connecting to MQTT broker");
  while (!mqtt_client.connected()) {
    if (mqtt_client.connect(MCU)) {
      Serial.println("\nMQTT connected");
    } else {
      Serial.print(".");
      delay(100);
    }
  }
}

void writeByteI2C(TwoWire &wire, uint8_t address, uint8_t subAddress, uint8_t data) {
  wire.beginTransmission(address);
  wire.write(subAddress);
  wire.write(data);
  wire.endTransmission();
}

void readBytesI2C(TwoWire &wire, uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest) {
  wire.beginTransmission(address);
  wire.write(subAddress);
  wire.endTransmission(false);
  uint8_t i = 0;
  wire.requestFrom(address, count);
  while (wire.available()) {
    dest[i++] = wire.read();
  }
}

String getISOTimestamp(time_t t) {
  struct tm *tm = localtime(&t);
  char buffer[20];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H-%M-%S", tm);
  return String(buffer);
}
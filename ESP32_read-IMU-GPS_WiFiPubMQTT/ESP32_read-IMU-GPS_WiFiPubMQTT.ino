// === Librerie ===
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ICM20948_WE.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// === WiFi === (smartphone's hotspot)
const char* ssid = "ssid";
const char* password = "pwd";

// === MQTT === (broker Mosquitto on Mac Air)
const char* mqtt_server = "192.168.137.4";  // broker's IP address
const char* mqtt_topic = "esp32/imugps";  // topic to publish data
WiFiClient espClient;
PubSubClient client(espClient);

// === ICM-20948 ===
#define ICM20948_ADDR 0x68  // I2C address for BMI160 (with SAO pin → disconnected or 3.3V), connect to GND for 0x68
#define IMU_SAMPLE_RATE 10
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

// === GPS M100-5883 ===
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
#define GPS_SAMPLE_RATE 1
#define GPS_RX_PIN 16     // D6(GPIO16) → RX
#define GPS_TX_PIN 17     // D7(GPIO17) → TX

#define BUFFER_SIZE 1       // [in seconds] are we gonna publish every 1 or 10 seconds?🤔

struct IMUData {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
};

struct GPSData {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint32_t time;
  double lat;
  double lng;
  int sat;
  float spd;
};

IMUData imuBuffer[IMU_SAMPLE_RATE*BUFFER_SIZE];
int imuIndex = 0;
GPSData gpsBuffer[GPS_SAMPLE_RATE*BUFFER_SIZE];
int gpsIndex = 0;
unsigned int lastPublish = 0;

// === Setup ===
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // GPS (Serial1)
  gpsSerial.begin(9600, SERIAL_8N1, GPS_TX_PIN, GPS_RX_PIN);
  Serial.println("\nGPS is setting up...");

  // ICM20948
  while(!myIMU.init()){
    Serial.println("ICM20948 does not respond");
    delay(100);
  }
  Serial.println("ICM20948 is connected");
  while(!myIMU.initMagnetometer()){
    Serial.println("Magnetometer does not respond");
    delay(500);
  }
  Serial.println("Magnetometer is connected");
  config_ICM_20948();

  // WiFi
  Serial.print("WiFi is setting up..");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi connected");
  // MQTT
  client.setServer(mqtt_server, 1883);
  connectMQTT();
  
  // check GPS fix (wait up to 60sec)
  for (int triesLeft = 60; !(gps.location.isValid() && gps.satellites.value()>=3) && triesLeft>0; triesLeft--) {
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
    Serial.print("📡 Sat: ");
    Serial.print(gps.satellites.value());
    Serial.print(" | ✅ Fix: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", ");
    Serial.print(gps.location.lng(), 6);
  }

  
  lastPublish = millis();
}

// === Loop ===
void loop() {
  static unsigned long lastTimeMeasure = 0;
  if (millis() - lastTimeMeasure >= 1000 / IMU_SAMPLE_RATE) {
    readIMU();
    imuIndex++;
    lastTimeMeasure=millis();
    if (imuIndex % (IMU_SAMPLE_RATE/GPS_SAMPLE_RATE) == 1) {
      readGPS();
      gpsIndex++;
    }
  }

  if (millis() - lastPublish >= BUFFER_SIZE*1000) {
    publishData();
    imuIndex = 0;
    gpsIndex = 0;
    lastPublish = millis();
  }
}

void config_ICM_20948(){
  /******************* Calibration ******************/ 
  
  /*  This is a method to calibrate. You have to determine the minimum and maximum 
   *  raw acceleration values of the axes determined in the range +/- 2 g. 
   *  You call the function as follows: setAccOffsets(xMin,xMax,yMin,yMax,zMin,zMax);
   *  The parameters are floats. 
   *  The calibration changes the slope / ratio of raw acceleration vs g. The zero point is 
   *  set as (min + max)/2.
   */
  myIMU.setAccOffsets(-16330.0, 16450.0, -16600.0, 16180.0, -16520.0, 16690.0);
    
  /* The starting point, if you position the ICM20948 flat, is not necessarily 0g/0g/1g for x/y/z. 
   * The autoOffset function measures offset. It assumes your ICM20948 is positioned flat with its 
   * x,y-plane. The more you deviate from this, the less accurate will be your results.
   * It overwrites the zero points of setAccOffsets, but keeps the correction of the slope.
   * The function also measures the offset of the gyroscope data. The gyroscope offset does not   
   * depend on the positioning.
   * This function needs to be called after setAccOffsets but before other settings since it will 
   * overwrite settings!
   * You can query the offsets with the functions:
   * xyzFloat getAccOffsets() and xyzFloat getGyrOffsets()
   * You can apply the offsets using:
   * setAccOffsets(xyzFloat yourOffsets) and setGyrOffsets(xyzFloat yourOffsets)
   */
  Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
  delay(500);
  myIMU.autoOffsets();
  delay(500);
  Serial.println("Done!"); 
  
  /*  The gyroscope data is not zero, even if you don't move the ICM20948. 
   *  To start at zero, you can apply offset values. These are the gyroscope raw values you obtain
   *  using the +/- 250 degrees/s range. 
   *  Use either autoOffset or setGyrOffsets, not both.
   */
  //myIMU.setGyrOffsets(-115.0, 130.0, 105.0);
  
  /*  ICM20948_ACC_RANGE_2G      2 g   (default)
   *  ICM20948_ACC_RANGE_4G      4 g
   *  ICM20948_ACC_RANGE_8G      8 g   
   *  ICM20948_ACC_RANGE_16G    16 g
   */
  myIMU.setAccRange(ICM20948_ACC_RANGE_16G);
  
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
  //myIMU.setAccDLPF(ICM20948_DLPF_OFF);    
  
  /*  Acceleration sample rate divider divides the output rate of the accelerometer.
   *  Sample rate = Basic sample rate / (1 + divider) 
   *  It can only be applied if the corresponding DLPF is not off!
   *  Divider is a number 0...4095 (different range compared to gyroscope)
   *  If sample rates are set for the accelerometer and the gyroscope, the gyroscope
   *  sample rate has priority.
   */
  //myIMU.setAccSampleRateDivider(100);
  
  /*  ICM20948_GYRO_RANGE_250       250 degrees per second (default)
   *  ICM20948_GYRO_RANGE_500       500 degrees per second
   *  ICM20948_GYRO_RANGE_1000     1000 degrees per second
   *  ICM20948_GYRO_RANGE_2000     2000 degrees per second
   */
  myIMU.setGyrRange(ICM20948_GYRO_RANGE_1000);
  
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
  //myIMU.setGyrDLPF(ICM20948_DLPF_OFF);  
  
  /*  Gyroscope sample rate divider divides the output rate of the gyroscope.
   *  Sample rate = Basic sample rate / (1 + divider) 
   *  It can only be applied if the corresponding DLPF is not OFF!
   *  Divider is a number 0...255
   *  If sample rates are set for the accelerometer and the gyroscope, the gyroscope
   *  sample rate has priority.
   */
  //myIMU.setGyrSampleRateDivider(100);

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
  //myIMU.setTempDLPF(ICM20948_DLPF_OFF);
 
  /* You can set the following modes for the magnetometer:
   * AK09916_PWR_DOWN          Power down to save energy
   * AK09916_TRIGGER_MODE      Measurements on request, a measurement is triggered by 
   *                           calling setMagOpMode(AK09916_TRIGGER_MODE)
   * AK09916_CONT_MODE_10HZ    Continuous measurements, 10 Hz rate
   * AK09916_CONT_MODE_20HZ    Continuous measurements, 20 Hz rate
   * AK09916_CONT_MODE_50HZ    Continuous measurements, 50 Hz rate
   * AK09916_CONT_MODE_100HZ   Continuous measurements, 100 Hz rate (default)
   */
  myIMU.setMagOpMode(AK09916_CONT_MODE_10HZ);
  delay(100); // add a delay of at least 1000/magRate to avoid first mag value being zero 
}

void readIMU() {
  xyzFloat gValue;
  xyzFloat degValue;
  xyzFloat magValue;
  
  myIMU.readSensor();
  myIMU.getGValues(&gValue);
  myIMU.getGyrValues(&degValue);
  myIMU.getMagValues(&magValue);
  float resultantG = myIMU.getResultantG(&gValue);

  if (imuIndex < IMU_SAMPLE_RATE*BUFFER_SIZE) {
    imuBuffer[imuIndex] = {
      gValue.x, gValue.y, gValue.z,
      degValue.x, degValue.y, degValue.z,
      magValue.x, magValue.y, magValue.z
    };
  }
}

void readGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()) {
      gpsBuffer[gpsIndex].year = gps.date.year();
      gpsBuffer[gpsIndex].month = gps.date.month();
      gpsBuffer[gpsIndex].day = gps.date.day();
      gpsBuffer[gpsIndex].time = gps.time.value();
      gpsBuffer[gpsIndex].lat = gps.location.lat();
      gpsBuffer[gpsIndex].lng = gps.location.lng();
      gpsBuffer[gpsIndex].sat = gps.satellites.value();
      gpsBuffer[gpsIndex].spd = gps.speed.kmph();
    }
  }
}

void publishData() {
  if (!client.connected()) {
    connectMQTT();
  }

  String payload = "{";
  payload += "\"imu\": [";
  for (int i = 0; i < imuIndex; i++) {
    payload += "\n\t{";
    payload += "\"ax\":" + String(imuBuffer[i].ax, 2) + ",";
    payload += "\"ay\":" + String(imuBuffer[i].ay, 2) + ",";
    payload += "\"az\":" + String(imuBuffer[i].az, 2) + ",";
    payload += "\"gx\":" + String(imuBuffer[i].gx, 2) + ",";
    payload += "\"gy\":" + String(imuBuffer[i].gy, 2) + ",";
    payload += "\"gz\":" + String(imuBuffer[i].gz, 2) + ",";
    payload += "\"mx\":" + String(imuBuffer[i].mx, 2) + ",";
    payload += "\"my\":" + String(imuBuffer[i].my, 2) + ",";
    payload += "\"mz\":" + String(imuBuffer[i].mz, 2) + "}";
    if (i < imuIndex - 1) payload += ",";
  }
  payload += "],\n\"gps\": [";
  for (int i = 0; i < gpsIndex; i++) {
    payload += "\n\t{";
    payload += "\"year\": " + String(gpsBuffer[i].year) + ",";
    payload += "\"month\": " + String(gpsBuffer[i].month) + ",";
    payload += "\"day\": " + String(gpsBuffer[i].day) + ",";
    payload += "\"time\": " + String(gpsBuffer[i].time) + ",";
    payload += "\"lat\": " + String(gpsBuffer[i].lat, 6) + ",";
    payload += "\"lng\": " + String(gpsBuffer[i].lng, 6) + ",";
    payload += "\"sat\": " + String(gpsBuffer[i].sat) + ",";
    payload += "\"spd\": " + String(gpsBuffer[i].spd, 2) + "}";
    if (i < imuIndex - 1) payload += ",";
  }
  payload += "]}";

  client.publish(mqtt_topic, payload.c_str(), true);
  Serial.println(payload);
}

void connectMQTT() {
  Serial.print("Connecting to MQTT broker..");
  while (!client.connected()) {
    if (client.connect("ESP32")) {
      Serial.println("\nMQTT connected");
    } else {
      Serial.print(".");
      delay(500);
    }
  }
}
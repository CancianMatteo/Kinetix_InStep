// === Librerie ===
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ICM_20948.h>  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
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
ICM_20948_I2C imu20948;
#define AD0_VAL 0  // last bit of I2C address of the IMU (normally and AD0 connected to 3.3V is 1, or 0 if AD0 connected to GND)
#define IMU_SAMPLE_RATE 10

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
  delay(500);
  while (imu20948.begin(Wire, AD0_VAL) != ICM_20948_Stat_Ok) {
    Serial.println("ICM IMU not detected!");
    delay(500);
  }
  Serial.println("ICM20948 is connected");
  //config_ICM_20948();

  // WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("WiFi connected");
  // MQTT
  client.setServer(mqtt_server, 1883);
  connectMQTT();
  
  // check GPS fix (wait up to 60sec)
  for (int triesLeft = 10; !(gps.location.isValid() && gps.satellites.value()>=3) && triesLeft>0; triesLeft--) {
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
  // Here we are doing a SW reset to make sure the device starts in a known state
  imu20948.swReset();
  delay(250);

  // Now wake the sensor up
  imu20948.sleep(false);
  imu20948.lowPower(false);

  // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

  /* Set Gyro and Accelerometer sample mode:
   *  ICM_20948_Sample_Mode_Continuous, performance mode continuous sampling;
   *  ICM_20948_Sample_Mode_Cycled, sample at given frequency, good for very low power but may be less accurate.
   */
  imu20948.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm16; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e: gpm2 / gpm4 / gpm8 / gpm16
  myFSS.g = dps1000; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e: dps250 / dps500 / dps1000 / dps2000

  imu20948.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  if (imu20948.status != ICM_20948_Stat_Ok){
    Serial.print("up to setFullScale returned: ");
    Serial.println(imu20948.statusString());
  }

  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                  // acc_d473bw_n499bw
                                  // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                  // acc_d111bw4_n136bw
                                  // acc_d50bw4_n68bw8
                                  // acc_d23bw9_n34bw4
                                  // acc_d11bw5_n17bw
                                  // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz

  myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                    // gyr_d361bw4_n376bw5
                                    // gyr_d196bw6_n229bw8
                                    // gyr_d151bw8_n187bw6
                                    // gyr_d119bw5_n154bw3
                                    // gyr_d51bw2_n73bw3
                                    // gyr_d23bw9_n35bw9
                                    // gyr_d11bw6_n17bw8
                                    // gyr_d5bw7_n8bw9

  imu20948.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);

  // Choose whether or not to use DLPF
  // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
  ICM_20948_Status_e accDLPEnableStat = imu20948.enableDLPF(ICM_20948_Internal_Acc, false);
  ICM_20948_Status_e gyrDLPEnableStat = imu20948.enableDLPF(ICM_20948_Internal_Gyr, false);

  // Choose whether or not to start the magnetometer
  imu20948.startupMagnetometer();
  delay(3000);
  if (imu20948.status != ICM_20948_Stat_Ok){
    Serial.println(imu20948.statusString());
    Serial.println("ICM-20948 configuration complete!");
  }else{
    Serial.println("ICM-20948 configuration failed successfully!");   // cit. Windows
  }
}

void readIMU() {
  while (!imu20948.dataReady());
  imu20948.getAGMT();
  if (imuIndex < IMU_SAMPLE_RATE*BUFFER_SIZE) {
    imuBuffer[imuIndex] = {
      imu20948.accX(), imu20948.accY(), imu20948.accZ(),
      imu20948.gyrX(), imu20948.gyrY(), imu20948.gyrZ(),
      imu20948.magX(), imu20948.magY(), imu20948.magZ()
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
    payload += "{";
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
  payload += "],\"gps\": [";
  for (int i = 0; i < gpsIndex; i++) {
    payload += "{";
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
      delay(100);
    }
  }
}
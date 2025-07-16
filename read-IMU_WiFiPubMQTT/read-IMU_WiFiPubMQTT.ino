// === Libraries ===
#include "credentials.h"
#include <WiFi.h>
#include <Wire.h>
#include "BMM350.h"
#include <ArduinoJson.h>
#include <time.h>

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

#define ASYNC_TCP_SSL_ENABLED true
#include <AsyncMQTT_ESP32.h>

// === WiFi === (smartphone's hotspot)
const char* ssid = WIFI_SSID; 
const char* password = WIFI_PASSWORD;

// === MQTT === (broker Mosquitto on Mac Air)
const char* mqtt_server = "192.168.137.212";  // broker's IP address
String mqtt_topic = "calcio/prova";    // topic to publish data

// SSL Configuration for secure config reception
#define MQTT_SSL_PORT 8883
#define MQTT_STANDARD_PORT 1883
const uint8_t MQTT_SERVER_FINGERPRINT[] = {0x7e, 0x36, 0x22, 0x01, 0xf9, 0x7e, 0x99, 0x2f, 0xc5, 0xdb, 0x3d, 0xbe, 0xac, 0x48, 0x67, 0x5b, 0x5d, 0x47, 0x94, 0xd2};

// MQTT Clients - one for secure config, one for fast data publishing
AsyncMqttClient mqttClientSSL;    // For secure configuration
AsyncMqttClient mqttClientData;   // For fast data publishing
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

// === Configuration ===
bool configReceived = false;
String efuseMacStr = String((uint64_t)ESP.getEfuseMac(), HEX);
float hardIron[3] = {0, 0, 0};
float softIron[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
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

#define IMU_SAMPLE_RATE 20     // advised not to exceed 100Hz
#define PUBLISH_INTERVAL 5      // [in seconds] publish every n second
// IMU_SAMPLE_RATE * PUBLISH_INTERVAL ≤ 100 to avoid negative impact on performance (example 20Hz * 5s = 100 samples 👍)
// going above 100 samples will most probably cause issues and delays with MQTT publish due to large payload size

#define BMI160_I2C_ADDRESS 0x69
#define BMM350_I2C_ADDRESS 0x14

BMM350 magnetometer(BMM350_I2C_ADDRESS); // or 0x15
float aRes, gRes;

struct IMUData {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
};

IMUData imuBuffer[IMU_SAMPLE_RATE * PUBLISH_INTERVAL];
int imuIndex = 0;
unsigned long nextSampleTime;
String payload = ""; // Payload to be published via MQTT
unsigned int nPublish = 0;

// === Setup ===
void setup() {
  pinMode(BUILTIN_USER_LED, OUTPUT);  // NB: XIAO use active low configuration for the user LED
  digitalWrite(BUILTIN_USER_LED, LOW); // Turn on the LED while configuring
  Serial.begin(115200);
  Wire.begin();

  // Initialize BMM350
  while (!magnetometer.begin(&Wire)) {
    Serial.println("Failed to initialize BMM350! Check your wiring.");
    delay(500);
  }
  magnetometer.setRateAndPerformance(BMM350_DATA_RATE_100HZ, BMM350_ULTRALOWNOISE);

  // Initialize BMI160
  BMI160_begin();
  autoCalibrateBMI160();

  aRes = 16.f / 32768.f;  // Accelerometer resolution
  gRes = 1000.f / 32768.f; // Gyroscope resolution

  Serial.print("Free heap: ");
  Serial.println(ESP.getFreeHeap());

  // Setup timers
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0,
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToMqttSSL));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0,
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  // Setup WiFi event handler
  WiFi.onEvent(WiFiEvent);

  // Setup MQTT SSL client for configuration
  mqttClientSSL.onConnect(onMqttSSLConnect);
  mqttClientSSL.onDisconnect(onMqttSSLDisconnect);
  mqttClientSSL.onMessage(onMqttSSLMessage);
  mqttClientSSL.setServer(mqtt_server, MQTT_SSL_PORT);
  mqttClientSSL.setSecure(true);
  mqttClientSSL.addServerFingerprint((const uint8_t *)MQTT_SERVER_FINGERPRINT);

  // Setup MQTT Data client for IMU publishing
  mqttClientData.onConnect(onMqttDataConnect);
  mqttClientData.onDisconnect(onMqttDataDisconnect);
  mqttClientData.setServer(mqtt_server, MQTT_STANDARD_PORT);
  mqttClientData.setSecure(false);

  // Start WiFi connection
  connectToWifi();

  // Wait for configuration
  waitForConfig();

  configTimeWithNTP();

  digitalWrite(BUILTIN_USER_LED, HIGH); // Turn off the LED after configuration

  delay(100); // Give some time before starting the loop

  Serial.println("Setup complete. Ready to read IMU data.");
  nextSampleTime = millis();
}

// === Loop ===
void loop() {
  // Time to sample?
  if (millis() >= nextSampleTime) {
    // Check if we missed more than one sample period
    int missedSamples = (millis() - nextSampleTime) / (1000 / IMU_SAMPLE_RATE);
    if (missedSamples > 0) {
      Serial.printf("Warning⚠️: Missed %d samples\n", missedSamples);
    }
    
    // Read IMU data
    readIMU();
    
    nextSampleTime += (1000 / IMU_SAMPLE_RATE) * (missedSamples+1);   // Schedule next sample time (POLICY: sample losts are not recovered)
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
  if (imuIndex == IMU_SAMPLE_RATE * PUBLISH_INTERVAL) {
    digitalWrite(BUILTIN_USER_LED, LOW);
    
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
    payload = "";
    digitalWrite(BUILTIN_USER_LED, HIGH);
  }

  if (nextSampleTime > millis())
    delay(nextSampleTime - millis()); // Wait until next sample time
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
  writeByteI2C(Wire, BMI160_I2C_ADDRESS, 0x41, 0x0C);  // Accelerometer range ±16G
  writeByteI2C(Wire, BMI160_I2C_ADDRESS, 0x43, 0x01);  // Gyroscope range ±1000dps
}

void autoCalibrateBMI160() {
    writeByteI2C(Wire, BMI160_I2C_ADDRESS, 0x69, 0x6F); // Write 0b01101111 (0b0gxxyyzz, see BMI160 datasheet pg. 78)
    writeByteI2C(Wire, BMI160_I2C_ADDRESS, 0x7E, 0x37); // Start accelerometer offset calibration
    writeByteI2C(Wire, BMI160_I2C_ADDRESS, 0x7E, 0x03); // Trigger auto-calibration (issuing start_foc command)
    // Wait for calibration to complete
    delay(500);
}

// === Get Config via MQTT ===

void waitForConfig() {
  Serial.print(efuseMacStr);
  Serial.println(" waiting for config on topic init/config...");

  unsigned long startTime = millis();
  while (!configReceived) {
    delay(100);
    if (millis() - startTime > 100000) { // Timeout after 100 seconds
      Serial.println("Config not received, proceeding with default values.");
      break;
    }
  }
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

// === Read IMU Data ===
void readIMU() {
  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, mx = 0, my = 0, mz = 0;

  readAccelGyroData(ax, ay, az, gx, gy, gz);
  magnetometer.readMagnetometerData(mx, my, mz);

  if (imuIndex < IMU_SAMPLE_RATE * PUBLISH_INTERVAL) {
    imuBuffer[imuIndex] = {ax, ay, az, gx, gy, gz, mx, my, mz};
    imuIndex++;
  }
}

// === Build Message and Publish Data with MQTT ===
String buildMQTTPayload() {
  if (ESP.getFreeHeap() < 10000) {
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
  pld += "]";

  pld += "}}";
  return pld;
}

bool publishData(const String& localPayload) {
  if (!mqttClientData.connected()) {
    connectToMqttData();
    // Wait a bit for connection
    int attempts = 0;
    while (!mqttClientData.connected() && attempts < 50) {
      delay(100);
      attempts++;
    }
    if (!mqttClientData.connected()) {
      Serial.println("Failed to connect to MQTT Data broker");
      return false;
    }
  }

  uint16_t packetId = mqttClientData.publish(mqtt_topic.c_str(), 0, true, localPayload.c_str());
  if (packetId == 0) {
    Serial.println("Publish failed");
    return false;
  }
  
  return true;
}

// === Other Functions ===
void readAccelGyroData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  int16_t IMUCount[6];
  uint8_t rawData[12];

  readBytesI2C(Wire, BMI160_I2C_ADDRESS, 0x0C, 12, &rawData[0]);

  IMUCount[0] = ((int16_t)rawData[1] << 8) | rawData[0];
  IMUCount[1] = ((int16_t)rawData[3] << 8) | rawData[2];
  IMUCount[2] = ((int16_t)rawData[5] << 8) | rawData[4];
  IMUCount[3] = ((int16_t)rawData[7] << 8) | rawData[6];
  IMUCount[4] = ((int16_t)rawData[9] << 8) | rawData[8];
  IMUCount[5] = ((int16_t)rawData[11] << 8) | rawData[10];

  ax = (float)IMUCount[3] * aRes;
  ay = (float)IMUCount[4] * aRes;
  az = (float)IMUCount[5] * aRes;

  gx = (float)IMUCount[0] * gRes;
  gy = (float)IMUCount[1] * gRes;
  gz = (float)IMUCount[2] * gRes;
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

// === WiFi and MQTT Connection Functions ===
void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
}

void connectToMqttSSL() {
  Serial.println("Connecting to MQTT SSL...");
  mqttClientSSL.connect();
}

void connectToMqttData() {
  Serial.println("Connecting to MQTT Data...");
  mqttClientData.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
#if USING_CORE_ESP32_CORE_V200_PLUS
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqttSSL();
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      break;
#else
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqttSSL();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      break;
#endif
    default:
      break;
  }
}

// === MQTT SSL Callbacks (for configuration) ===
void onMqttSSLConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT SSL broker for configuration");
  mqttClientSSL.subscribe("init/config", 2);
  Serial.print(efuseMacStr);
  Serial.println(" waiting for config on topic init/config...");
}

void onMqttSSLDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT SSL");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSSLMessage(char* topic, char* conf_payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.print("SSL Message arrived on topic: ");
  Serial.println(topic);
  if (String(topic) == "init/config") {
    StaticJsonDocument<1024> doc;
    DeserializationError err = deserializeJson(doc, conf_payload, len);
    if (err) {
      Serial.println("JSON parse failed");
      return;
    }
    if (doc.containsKey(efuseMacStr)) {
      JsonObject obj = doc[efuseMacStr];
      if (obj.containsKey("mqtt_topic")) mqtt_topic = obj["mqtt_topic"].as<String>();
      if (obj.containsKey("hard_iron")) {
        for (int i = 0; i < 3; i++)
          hardIron[i] = obj["hard_iron"][i];
      }
      if (obj.containsKey("soft_iron")) {
        for (int i = 0; i < 3; i++)
          for (int j = 0; j < 3; j++)
            softIron[i][j] = obj["soft_iron"][i][j];
      }
      if (obj.containsKey("player")) player = obj["player"].as<String>();
      if (obj.containsKey("exercise_type")) exercise_type = obj["exercise_type"].as<String>();

      // Apply calibration to magnetometer
      magnetometer.setCalibration(hardIron, softIron);
      configReceived = true;
      Serial.println("Config received and applied.");
      
      // Disconnect SSL client and connect data client
      mqttClientSSL.disconnect();
      connectToMqttData();
    }
  }
}

// === MQTT Data Callbacks (for IMU data publishing) ===
void onMqttDataConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT Data broker for publishing");
}

void onMqttDataDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT Data");
  if (WiFi.isConnected()) {
    connectToMqttData();
  }
}

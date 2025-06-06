// === Libraries ===
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "BMM350.h"
#include <ArduinoJson.h>

// === WiFi === (smartphone's hotspot)
const char* ssid = "ssid";
const char* password = "pwd";

// === MQTT === (broker Mosquitto on Mac Air)
const char* mqtt_server = "ip";  // broker's IP address
String mqtt_topic = "calcio/prova";    // topic to publish data
WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);

// config
String efuseMacStr = String((uint64_t)ESP.getEfuseMac(), HEX);
float hardIron[3] = {0, 0, 0};
float softIron[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
String player, exercise_type;
unsigned long config_time = 0;
bool configReceived = false;

#define MCU "ESP32S3"
#define BUILTIN_USER_LED 21 // GPIO21 = LED giallo USER
// === BMI160 + BMM350 ===
#define BMI160_I2C_ADDRESS 0x69
#define BMM350_I2C_ADDRESS 0x14
#define IMU_SAMPLE_RATE 20
#define BUFFER_SIZE 5  // [in seconds] publish every n second

BMM350 magnetometer(BMM350_I2C_ADDRESS); // or 0x15
float aRes, gRes;

struct IMUData {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
};

IMUData imuBuffer[IMU_SAMPLE_RATE * BUFFER_SIZE];
int imuIndex = 0;
String payload = ""; // Payload to be published via MQTT
unsigned int nPublish = 0;

// === Setup ===
void setup() {
  pinMode(BUILTIN_USER_LED, OUTPUT);
  digitalWrite(BUILTIN_USER_LED, HIGH); // Turn on the LED while configuring
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
  autoCalibrateAccelerometer();

  aRes = 16.f / 32768.f;  // Accelerometer resolution
  gRes = 1000.f / 32768.f; // Gyroscope resolution

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
  mqtt_client.setBufferSize(60000);
  Serial.print("MQTT buffer size: ");
  Serial.println(mqtt_client.getBufferSize());
  mqtt_client.setServer(mqtt_server, 1883);
  mqtt_client.setKeepAlive(300);
  connectMQTT();

  waitForConfig();
  digitalWrite(BUILTIN_USER_LED, LOW); // Turn off the LED after configuration
}

// === Loop ===
void loop() {
  static unsigned long lastTimeMeasure = 0;
  if (millis() - lastTimeMeasure >= 1000 / IMU_SAMPLE_RATE) {
    if (millis() - lastTimeMeasure >= 2000 / IMU_SAMPLE_RATE) {
      Serial.println("Warning⚠️: can't keep up with required IMU sample rate, system overloaded!");
    }
    readIMU();
    lastTimeMeasure = millis();
  }

  // Publish when buffer is full
  if (imuIndex == IMU_SAMPLE_RATE * BUFFER_SIZE) {
    payload = buildMQTTPayload();
    digitalWrite(BUILTIN_USER_LED, HIGH); // Turn on the LED while publishing
    if (payload.length() > 0) {
      publishData(payload);
    } else {
      Serial.println("Failed to build payload, not publishing data.");
    }
    imuIndex = 0;
    payload = ""; // Clear payload after publishing
    digitalWrite(BUILTIN_USER_LED, LOW);
  }
}

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

void autoCalibrateAccelerometer() {
    // Configure accelerometer for auto-calibration
    Wire.beginTransmission(BMI160_I2C_ADDRESS);
    Wire.write(0x7E); // Command register
    Wire.write(0x37); // Start accelerometer offset calibration
    Wire.endTransmission();
    // Wait for calibration to complete
    delay(1000);
}

void callback(char* topic, byte* conf_payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  if (String(topic) == "init/config" && !configReceived) {
    StaticJsonDocument<1024> doc;
    DeserializationError err = deserializeJson(doc, conf_payload, length);
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
      if (obj.containsKey("time")) config_time = obj["time"].as<unsigned long>();

      // Apply calibration to magnetometer here if needed
      magnetometer.setCalibration(hardIron, softIron);
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

// === Read IMU Data ===
void readIMU() {
  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, mx = 0, my = 0, mz = 0;

  readAccelGyroData(ax, ay, az, gx, gy, gz);
  magnetometer.readMagnetometerData(mx, my, mz);

  if (imuIndex < IMU_SAMPLE_RATE * BUFFER_SIZE) {
    imuBuffer[imuIndex] = {ax, ay, az, gx, gy, gz, mx, my, mz};
    imuIndex++;
  }
}

// === Build Message and Publish Data with MQTT ===
String buildMQTTPayload() {
  if (ESP.getFreeHeap() < 60000) {
    Serial.println("Free heap memory is low, not publishing data");
    return "";
  }

  int estimatedSize = 500 + (imuIndex * 9 * 6); // 9 values * 6 chars avg per value
  String pld;
  pld.reserve(estimatedSize);
  pld = "{\"player\":\"" + player + "\", \"source\":\"" + MCU + "-" + efuseMacStr + "\", \"exercise_type\":\"" + exercise_type + "\", \"time\":\"" + String(config_time) + "\", ";
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
  unsigned long startTime = millis();

  if (!mqtt_client.connected()) {
    connectMQTT();
  }

  bool beginOk = mqtt_client.beginPublish(mqtt_topic.c_str(), localPayload.length(), true);
  if (!beginOk) {
    Serial.println("beginPublish failed");
    return false;
  }
  size_t written = mqtt_client.print(localPayload);
  bool endOk = mqtt_client.endPublish();
  if (!endOk) {
    Serial.println("endPublish failed");
    return false;
  }
  Serial.print(++nPublish);
  Serial.print("° message published (");
  Serial.print(millis() - startTime);
  Serial.print("ms): ");
  Serial.println(localPayload);

  return true;
}

// === Connect MQTT ===
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
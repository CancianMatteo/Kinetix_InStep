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
String mqtt_topic = "topic";    // topic to publish data
WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);

// config
String efuseMacStr;
float hardIron[3] = {0, 0, 0};
float softIron[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
String player, exercise_type;
unsigned long config_time = 0;
bool configReceived = false;

// === BMI160 + BMM350 ===
#define BMI160_I2C_ADDRESS 0x69
#define BMM350_I2C_ADDRESS 0x14
#define IMU_SAMPLE_RATE 100
#define BUFFER_SIZE 5  // [in seconds] publish every n second

#define BUILTIN_USER_LED 21 // GPIO21 = LED giallo USER

BMM350 magnetometer(BMM350_I2C_ADDRESS); // or 0x15
float aRes, gRes;

struct IMUData {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
};

IMUData imuBuffer[IMU_SAMPLE_RATE * BUFFER_SIZE];
int imuIndex = 0;
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

  // Perform accelerometer auto-calibration
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

  // MQTT
  mqtt_client.setServer(mqtt_server, 1883);
  mqtt_client.setKeepAlive(300);
  connectMQTT();
  mqtt_client.setBufferSize(65536);
  Serial.print("MQTT buffer size: ");
  Serial.println(mqtt_client.getBufferSize());

  waitForConfig();
  digitalWrite(BUILTIN_USER_LED, LOW); // Turn off the LED after configuration
}

// === Loop ===
void loop() {
  static unsigned long lastTimeMeasure = 0;
  if (millis() - lastTimeMeasure >= 1000 / IMU_SAMPLE_RATE) {
    readIMU();
    imuIndex++;
    lastTimeMeasure = millis();
  }

  if (imuIndex == IMU_SAMPLE_RATE * BUFFER_SIZE) {
    digitalWrite(BUILTIN_USER_LED, HIGH);
    publishData();
    imuIndex = 0;
    digitalWrite(BUILTIN_USER_LED, LOW);
  }
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

void callback(char* topic, byte* payload, unsigned int length) {
  if (String(topic) == "init/config" && !configReceived) {
    StaticJsonDocument<1024> doc;
    DeserializationError err = deserializeJson(doc, payload, length);
    if (err) {
      Serial.println("JSON parse failed");
      return;
    }
    if (!doc.containsKey(efuseMacStr)) return;
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

void waitForConfig() {
  efuseMacStr = String((uint64_t)ESP.getEfuseMac(), HEX);
  mqtt_client.setCallback(callback);
  mqtt_client.subscribe("init/config");
  Serial.println("Waiting for config on topic init/config...");
  while (!configReceived) {
    mqtt_client.loop();
    delay(10);
  }
  mqtt_client.unsubscribe("init/config");
  mqtt_client.setCallback(NULL); // Remove callback if not needed anymore
}

void readIMU() {
  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, mx = 0, my = 0, mz = 0;

  readAccelGyroData(ax, ay, az, gx, gy, gz);
  magnetometer.readMagnetometerData(mx, my, mz);

  if (imuIndex < IMU_SAMPLE_RATE * BUFFER_SIZE) {
    imuBuffer[imuIndex] = {ax, ay, az, gx, gy, gz, mx, my, mz};
  }
}

bool publishData() {
  if (!mqtt_client.connected()) {
    connectMQTT();
  }

  if (ESP.getFreeHeap() < 66000) {
    Serial.println("Free heap memory is low, not publishing data");
    return false;
  }

  String payload = "{\"player\":\"" + player + "\",";
  payload += "\"exercise_type\":\"" + exercise_type + "\",";
  payload += "\"time\":" + String(config_time) + ",";
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
  payload += "]}";

  do {
    mqtt_client.beginPublish(mqtt_topic.c_str(), payload.length(), true);
    mqtt_client.print(payload);
  } while (!mqtt_client.endPublish());
  Serial.print(++nPublish);
  Serial.println("° message published successfully:");
  Serial.println(payload);

  return true;
}

void connectMQTT() {
  Serial.print("Connecting to MQTT broker");
  while (!mqtt_client.connected()) {
    if (mqtt_client.connect("ESP32")) {
      Serial.println("\nMQTT connected");
    } else {
      Serial.print(".");
      delay(100);
    }
  }
}

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
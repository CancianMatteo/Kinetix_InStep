#include <WiFi.h>
#include <PubSubClient.h>

// Replace with your network credentials
const char* ssid = "PC-MATTEO";
const char* password = "matteooo";

// Replace with your MQTT broker info
const char* mqtt_server = "192.168.137.212";
const int mqtt_port = 1883;
const char* topic = "onPower";

WiFiClient espClient;
PubSubClient client(espClient);

const int USER_LED = 21;  // GPIO21 = LED giallo USER

String clientId;

void setup_wifi() {
  Serial.print("Connecting WiFi ");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println("Connected");
  Serial.printf("%s: %d dBm\n", WiFi.SSID().c_str(), WiFi.RSSI());
}

void reconnect() {
  Serial.print("Connecting MQTT ");
  while (!client.connected()) {
    clientId = "ESP32S3-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (client.connect(clientId.c_str())) {
      String message = "Hi, I'm "+clientId;
      client.publish(topic, message.c_str());
      Serial.println("Connected");
    } else {
      Serial.print(".");
      delay(500);
    }
  }
}

void setup() {
  pinMode(USER_LED, OUTPUT);
  digitalWrite(USER_LED, LOW);

  Serial.begin(115200);
  delay(100);
  Serial.println("Starting...");

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  reconnect();
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  Serial.println("Hi, I'm "+clientId);

  // Blink USER LED
  digitalWrite(USER_LED, HIGH);
  delay(300);
  digitalWrite(USER_LED, LOW);
  delay(300);
}
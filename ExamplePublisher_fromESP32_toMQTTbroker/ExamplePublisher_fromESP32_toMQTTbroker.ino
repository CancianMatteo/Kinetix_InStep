#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "wifi-ssid";
const char* password = "wifi-pwd";

const char* mqtt_server = "192.168.137.4";  // Your MQTT broker IP
WiFiClient espClient;
PubSubClient client(espClient);
float temperature=20, humidity=60;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }

  Serial.println("WiFi connected");

  client.setServer(mqtt_server, 1883);
  while (!client.connected()) {
    if (client.connect("ESP32")) {
      Serial.println("MQTT connected");
    } else {
      delay(500);
    }
  }
}

void loop() {
  if (!client.connected()) {
    client.connect("ESP32");
    Serial.println("MQTT reconnected");
  }

  String payload = "{\"temperature\":" + String(temperature) + ",\"humidity\":" + String(humidity) + "}";
  client.publish("esp32/example", payload.c_str(), true); //retained=true
  humidity += 1;  // Simulate changing humidity
  temperature += 0.1;  // Simulate changing temperature
  delay(3000);  // publish every 3s
}
 
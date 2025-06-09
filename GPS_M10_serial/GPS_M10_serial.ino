#include <TinyGPSPlus.h>

// Nologo ESP32C3 SuperMini
#define GPS_RX_PIN 21     // TX(GPIO21) → RX
#define GPS_TX_PIN 20     // RX(GPIO20) → TX

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // Use UART1 on ESP32

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(115200, SERIAL_8N1, GPS_TX_PIN, GPS_RX_PIN);
  Serial.println("⏳ Waiting for GPS fix...");
}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Print progress every 2 seconds
  if (millis() - lastPrint > 2000) {
    lastPrint = millis();

    if (gps.satellites.isValid()) {
      Serial.print("📡 Satellites: ");
      Serial.print(gps.satellites.value());
    } else {
      Serial.print("📡 Satellites: --");
    }

    if (gps.location.isValid()) {
      if (gps.satellites.value() >= 3){
        Serial.print(" | ✅ Updated Fix: ");
      }else{
        Serial.print(" | ⚠️ Obsolete Fix: ");
      }
      Serial.print(gps.location.lat(), 6);
      Serial.print(", ");
      Serial.print(gps.location.lng(), 6);
    } else {
      Serial.print(" | ❌ No Fix Yet");
    }

    Serial.println();
  }
}

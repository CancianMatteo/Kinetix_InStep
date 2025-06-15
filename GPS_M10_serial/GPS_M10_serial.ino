#include <TinyGPSPlus.h>

// Nologo ESP32C3 SuperMini
#define GPS_RX_PIN 21     // TX(GPIO21) → RX
#define GPS_TX_PIN 20     // RX(GPIO20) → TX

#define GPS_SAMPLE_RATE 10

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // Use UART1 on ESP32

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(115200, SERIAL_8N1, GPS_TX_PIN, GPS_RX_PIN);

  // Configure GPS for 10Hz update rate with NMEA messages only
  configureGPS10Hz();
  delay(500);
  // gpsSerial.setRxBufferSize(1024);   // Optional: Increase serial buffer size

  Serial.println("⏳ Waiting for GPS fix...");
}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Print progress every 100 ms
  if (millis() - lastPrint > 100) {
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

void configureGPS10Hz() {
  Serial.print("Configuring GPS for 10Hz output (NMEA only)... ");
  
  // Disable UBX messages first
  // CFG-MSG message to disable specific UBX message types
  uint8_t disableUBX[] = {
    0xB5, 0x62,           // Header
    0x06, 0x01,           // Class/ID (CFG-MSG)
    0x03, 0x00,           // Payload length
    0x01, 0x00, 0x00,     // Disable UBX-NAV-POSLLH
    0x0B, 0x38            // Checksum
  };
  
  // Set measurement rate to 100ms (10Hz)
  uint8_t setRate[] = {
    0xB5, 0x62,           // Header
    0x06, 0x08,           // Class/ID (CFG-RATE)
    0x06, 0x00,           // Payload length
    0x64, 0x00,           // Measurement rate in ms (100ms = 10Hz)
    0x01, 0x00,           // Navigation rate (1 = use every measurement)
    0x01, 0x00,           // Time reference (1 = GPS time)
    0x7A, 0x12            // Checksum
  };

  // Enable NMEA GGA message (Position data)
  uint8_t enableGGA[] = {
    0xB5, 0x62,           // Header
    0x06, 0x01,           // Class/ID (CFG-MSG)
    0x08, 0x00,           // Payload length
    0xF0, 0x00,           // Message: NMEA-GGA
    0x00, 0x00,           // Ports: I2C
    0x00, 0x01,           // Ports: UART1 (enabled)
    0x00, 0x00,           // Ports: USB & SPI
    0x00, 0x01            // Checksum
  };

  // Enable NMEA RMC message (Position, velocity, time data)
  uint8_t enableRMC[] = {
    0xB5, 0x62,           // Header
    0x06, 0x01,           // Class/ID (CFG-MSG)
    0x08, 0x00,           // Payload length
    0xF0, 0x04,           // Message: NMEA-RMC
    0x00, 0x00,           // Ports: I2C
    0x00, 0x01,           // Ports: UART1 (enabled)
    0x00, 0x00,           // Ports: USB & SPI
    0x04, 0x1D            // Checksum
  };

  // Send all configuration commands
  for (int i = 0; i < sizeof(disableUBX); i++) {
    gpsSerial.write(disableUBX[i]);
  }
  delay(50);

  for (int i = 0; i < sizeof(setRate); i++) {
    gpsSerial.write(setRate[i]);
  }
  delay(50);

  for (int i = 0; i < sizeof(enableGGA); i++) {
    gpsSerial.write(enableGGA[i]);
  }
  delay(50);

  for (int i = 0; i < sizeof(enableRMC); i++) {
    gpsSerial.write(enableRMC[i]);
  }
  gpsSerial.flush(); // Ensure all data is sent
  delay(100);
  
  // Save configuration to make it persistent
  uint8_t saveCfg[] = {
    0xB5, 0x62,           // Header
    0x06, 0x09,           // Class/ID (CFG-CFG)
    0x0D, 0x00,           // Payload length
    0x00, 0x00, 0x00, 0x00, // Clear mask
    0xFF, 0xFF, 0x00, 0x00, // Save mask (save all)
    0x00, 0x00, 0x00, 0x00, // Load mask
    0x01,                 // Device mask (BBR/FLASH)
    0x1B, 0xAB            // Checksum
  };
  
  for (int i = 0; i < sizeof(saveCfg); i++) {
    gpsSerial.write(saveCfg[i]);
  }

  gpsSerial.flush(); // Ensure all data is sent
  Serial.println("GPS config completed.");
}

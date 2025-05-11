#include <Wire.h>
void setup() {}

void loop() {
  Wire.begin(22, 23); // change to actual SDA/SCL used
  Serial.begin(115200);
  delay(100);
  Serial.println("Scanning...");
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at 0x");
      Serial.println(addr, HEX);
    }
  }
  Serial.println("Done.");
  delay(2000);
}
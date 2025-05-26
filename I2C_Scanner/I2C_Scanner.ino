#include <Arduino.h>
#include <Wire.h>

void setup() {
  Wire.begin(8, 9); // change to actual SDA, SCL used
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  Serial.print("Scanning");
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at 0x");
      Serial.println(addr, HEX);
    }else{
      Serial.print(".");
    }
    delay(20);
  }
  Serial.println("Done");
  delay(2000);
}
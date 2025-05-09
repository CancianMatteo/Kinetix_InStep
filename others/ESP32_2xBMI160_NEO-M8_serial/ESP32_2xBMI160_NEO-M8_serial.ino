#include <Wire.h>
#include <TinyGPSPlus.h>
 
// I2C Configuration for XIAO ESP32C6
#define BMI160_ADDR_1 0x68  // I2C address for the first BMI160 (SAO low)
#define BMI160_ADDR_2 0x69  // I2C address for the second BMI160 (SAO high)
#define ESP32_SDA_PIN 22     // I2C D4(GPIO22) → SDA
#define ESP32_SCL_PIN 23     // I2C D5(GPIO23) → SCL
#define ACCEL_SENSITIVITY 16384.0 // Sensitivity for ±2g in LSB/g (adjust based on your configuration)

#define GPS_RX_PIN 16     // D6(GPIO16) → RX
#define GPS_TX_PIN 17     // D7(GPIO17) → TX

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // Use UART1 on ESP32

void setup() {
  Serial.begin(115200); // Initialize Serial communication
  Wire.begin(ESP32_SDA_PIN, ESP32_SCL_PIN);         // Initialize I2C communication
 
  // Initialize BMI160 accelerometer
  Wire.beginTransmission(BMI160_ADDR_1);
  Wire.write(0x7E); // Command register
  Wire.write(0x11); // Set accelerometer to normal mode
  Wire.endTransmission();
  delay(100);
  // Perform accelerometer auto-calibration
  autoCalibrateAccelerometer(BMI160_ADDR_1);

  // Initialize BMI160 accelerometer
  Wire.beginTransmission(BMI160_ADDR_2);
  Wire.write(0x7E); // Command register
  Wire.write(0x11); // Set accelerometer to normal mode
  Wire.endTransmission();
  delay(100);
  // Perform accelerometer auto-calibration
  autoCalibrateAccelerometer(BMI160_ADDR_2);
  
  Serial.println("BMI160 and BMM150 Initialized and Calibrated");

  gpsSerial.begin(9600, SERIAL_8N1, GPS_TX_PIN, GPS_RX_PIN);
  Serial.println("Waiting for GPS lock... ⏳");
}
 
void loop() {
  // ============ ACCELEROMETER 1 read ============   //BMI160 soldered uspide down 😣
  int16_t ax, ay, az;
  Wire.beginTransmission(BMI160_ADDR_1);
  Wire.write(0x12); // Start register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(BMI160_ADDR_1, 6);
 
  if (Wire.available() == 6) {
    ax = (Wire.read() | (Wire.read() << 8));
    ay = -(Wire.read() | (Wire.read() << 8));
    az = -(Wire.read() | (Wire.read() << 8));
  }
  // pitch-90: -84.7, roll-90: -91.0; | pitch-45: -45.7, roll-45: -46.4; | pitch0: 0.45, roll0:-1; | pitch45: 43.7, roll45: 44.1; | pitch90: -84.7, roll90: 89.5; | pitch180: -84.7, roll180: 179.8

  // ============ ACCELEROMETER 2 read ============
  int16_t ax2, ay2, az2;
  Wire.beginTransmission(BMI160_ADDR_2);
  Wire.write(0x12); // Start register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(BMI160_ADDR_2, 6);
 
  if (Wire.available() == 6) {
    ax2 = (Wire.read() | (Wire.read() << 8));
    ay2 = (Wire.read() | (Wire.read() << 8));
    az2 = (Wire.read() | (Wire.read() << 8));
  }
  // pitch-90: -91.7, roll-90: -87.5; | pitch-45: -42.7, roll-45: -44.4; | pitch0: 0.65, roll0:-0.4; | pitch45: 45.2, roll45: 45; | pitch90: -84.7, roll90: 91.2; | pitch180: -84.7, roll180: -178.2

  // ================== GPS read ==================
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
 
  // ============ ACCELEROMETER 1 print ============
  // Convert raw accelerometer values to m/s^2
  float ax_mps2 = ax /ACCEL_SENSITIVITY *9.81;
  float ay_mps2 = ay /ACCEL_SENSITIVITY *9.81;
  float az_mps2 = az /ACCEL_SENSITIVITY *9.81;
 
  // Print accelerometer values in m/s^2
  Serial.print("Acceleration 1 (m/s^2): ");
  Serial.print(ax_mps2-0.2, 2);
  Serial.print(", ");
  Serial.print(ay_mps2+0.7, 2);
  Serial.print(", ");
  Serial.print(az_mps2, 2);
  Serial.print(". \t");

  // Convert raw accelerometer values to g
  float ax_g = ax / ACCEL_SENSITIVITY;
  float ay_g = ay / ACCEL_SENSITIVITY;
  float az_g = az / ACCEL_SENSITIVITY;
 
  // Calculate tilt angles (pitch and roll) in degrees
  float pitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI;
  float roll = atan2(ay_g, az_g) * 180.0 / PI;
 
  // Print tilt angles
  Serial.print("Pitch 1: ");
  Serial.print(pitch-4.3, 1);
  Serial.print("°, Roll 1: ");
  Serial.print(roll, 1);
  Serial.println("°");
 
  // ============ ACCELEROMETER 2 print ============
  // Convert raw accelerometer values to m/s^2
  float ax2_mps2 = ax2 /ACCEL_SENSITIVITY *9.81;
  float ay2_mps2 = ay2 /ACCEL_SENSITIVITY *9.81;
  float az2_mps2 = az2 /ACCEL_SENSITIVITY *9.81;
 
  // Print accelerometer values in m/s^2
  Serial.print("Acceleration 2 (m/s^2): ");
  Serial.print(ax2_mps2-0.1, 2);
  Serial.print(", ");
  Serial.print(ay2_mps2+0.7, 2);
  Serial.print(", ");
  Serial.print(az2_mps2-0.1, 2);
  Serial.print(". \t");

  // Convert raw accelerometer values to g
  float ax2_g = ax2 / ACCEL_SENSITIVITY;
  float ay2_g = ay2 / ACCEL_SENSITIVITY;
  float az2_g = az2 / ACCEL_SENSITIVITY;
 
  // Calculate tilt angles (pitch and roll) in degrees
  float pitch2 = atan2(-ax2_g, sqrt(ay2_g * ay2_g + az2_g * az2_g)) * 180.0 / PI;
  float roll2 = atan2(ay2_g, az2_g) * 180.0 / PI;
 
  // Print tilt angles
  Serial.print("Pitch 2: ");
  Serial.print(pitch2+4, 1);
  Serial.print("°, Roll 2: ");
  Serial.print(roll2+0.3, 1);
  Serial.println("°");


  // ================== GPS print ==================
  if (gps.satellites.isValid()) {
    Serial.print("📡 Satellites: ");
    Serial.print(gps.satellites.value());
  } else {
    Serial.print("📡 Satellites: --");
  }

  if (gps.location.isUpdated()) {
    if (gps.satellites.value() >= 3){
      Serial.print(" | ✅ Updated Fix: ");
    }else{
      Serial.print(" | ⚠️ Obsolete Fix: ");
    }
    Serial.print(gps.location.lat(), 7);
    Serial.print(", ");
    Serial.println(gps.location.lng(), 7);
    Serial.print("🏃 Speed: ");
    Serial.println(gps.speed.kmph());
  } else {
    Serial.println(" | ❌ No Fix Yet");
  }

  delay(1000);
}

void autoCalibrateAccelerometer(uint8_t addr) {
  // Configure accelerometer for auto-calibration
  Wire.beginTransmission(addr);
  Wire.write(0x7E); // Command register
  Wire.write(0x37); // Start accelerometer offset calibration
  Wire.endTransmission();
  delay(100);
 
  // Wait for calibration to complete
  delay(1000);
  Serial.println("Accelerometer Auto-Calibration Complete");
}
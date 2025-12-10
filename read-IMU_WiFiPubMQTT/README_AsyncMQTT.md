# IMU WiFi MQTT with SSL Configuration

This project has been refactored to use WiFiClientSecure with PubSubClient for SSL/TLS encryption during initial configuration and standard MQTT for fast IMU data publishing.

## Architecture

The system uses two separate MQTT connections:
1. **SSL/TLS connection** - For receiving initial configuration (secure)
2. **Standard connection** - For publishing IMU data (fast, unencrypted)

## Setup Instructions

### ESP32 Setup

1. Install the required libraries:
   - PubSubClient (standard Arduino library)
   - WiFiClientSecure (built-in ESP32 library)
   - BMM350 library
   - ArduinoJson

2. Update `credentials.h` with your WiFi credentials:
   ```cpp
   #define WIFI_SSID         "your-wifi-name"
   #define WIFI_PASSWORD     "your-wifi-password"
   ```

3. Update the MQTT broker IP address in the main sketch:
   ```cpp
   const char* mqtt_server = "192.168.137.212";  // Your broker IP
   ```

### Python Configuration Sender

1. Install required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Update the configuration in `mqtt_ssl_config_sender.py`:
   - Set your MQTT broker IP
   - Configure ESP32 MAC addresses and their settings
   - Adjust SSL settings if needed

3. Run the configuration sender:
   ```bash
   python mqtt_ssl_config_sender.py
   ```

## How It Works

1. **Startup**: ESP32 connects to WiFi and establishes SSL connection to MQTT broker
2. **Configuration**: ESP32 subscribes to `init/config` topic and waits for configuration
3. **Config Reception**: Python script sends encrypted JSON configuration via SSL
4. **Switch to Data Mode**: After receiving config, ESP32 disconnects from SSL and connects to standard MQTT for data publishing
5. **Data Publishing**: IMU data is published unencrypted for maximum speed

## Libraries Used

- **WiFiClientSecure**: For SSL/TLS connections (built-in ESP32)
- **PubSubClient**: For MQTT communication
- **ArduinoJson**: For JSON parsing
- **BMM350**: For magnetometer communication

## Security Notes

- Initial configuration uses SSL/TLS encryption with certificate validation disabled for testing
- IMU data publishing is unencrypted for performance
- For production use, enable proper SSL certificate validation
- The current setup uses self-signed certificates for testing

## Troubleshooting

- Ensure your MQTT broker supports both SSL (port 8883) and standard (port 1883) connections
- Check that the ESP32 MAC address in the Python script matches your device
- Verify SSL certificate configuration if connection fails
- Monitor serial output for connection status and errors
- If compilation fails, ensure all required libraries are installed

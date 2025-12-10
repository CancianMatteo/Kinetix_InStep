#!/usr/bin/env python3
"""
MQTT SSL Configuration Sender

This script sends encrypted configuration JSON messages to ESP32 devices via MQTT using SSL/TLS encryption.

The configuration includes:
- MQTT topic for data publishing
- Hard iron calibration values
- Soft iron calibration matrix
- Player name and exercise type
"""

import paho.mqtt.client as mqtt
import json
import ssl
import time
import sys

# MQTT Broker Configuration
MQTT_BROKER = "192.168.137.212"  # Replace with your broker IP
MQTT_PORT = 8883                 # SSL port
MQTT_TOPIC = "init/config"
MQTT_USERNAME = None             # Set if broker requires authentication
MQTT_PASSWORD = None             # Set if broker requires authentication

# SSL Configuration
SSL_CA_CERT = None               # Path to CA certificate (if needed)
SSL_CERT_FILE = None             # Path to client certificate (if needed)
SSL_KEY_FILE = None              # Path to client key (if needed)

# Configuration for ESP32 devices
ESP32_CONFIGS = {
    "70af9d9006e8": {
        "mqtt_topic": "calcio/prova",
        "hard_iron": [0.0, 0.0, 0.0],
        "soft_iron": [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ],
        "player": "Player1",
        "exercise_type": "salto"
    },
    "ec879990a994": {
        "mqtt_topic": "calcio/player2",
        "hard_iron": [10.5, -5.2, 8.1],
        "soft_iron": [
            [1.02, 0.01, 0.02],
            [0.01, 0.98, 0.01],
            [0.02, 0.01, 1.01]
        ],
        "player": "Player2",
        "exercise_type": "corsa"
    }
}

class MQTTSSLConfigSender:
    def __init__(self):
        self.client = mqtt.Client()
        self.connected = False
        self.setup_callbacks()
        self.setup_ssl()
        
    def setup_callbacks(self):
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_publish = self.on_publish
        self.client.on_log = self.on_log
        
    def setup_ssl(self):
        context = ssl.create_default_context(ssl.Purpose.SERVER_AUTH)
        
        # For self-signed certificates or testing, you might need to disable certificate verification
        # TODO: WARNING: This is not recommended for production use
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
        
        # If you have proper certificates, uncomment and configure these:
        # if SSL_CA_CERT:
        #     context.load_verify_locations(SSL_CA_CERT)
        # if SSL_CERT_FILE and SSL_KEY_FILE:
        #     context.load_cert_chain(SSL_CERT_FILE, SSL_KEY_FILE)
        
        self.client.tls_set_context(context)
        
        # Set username and password if required
        if MQTT_USERNAME and MQTT_PASSWORD:
            self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT broker with SSL")
            self.connected = True
        else:
            print(f"Failed to connect to MQTT broker. Error code: {rc}")
            self.connected = False
    
    def on_disconnect(self, client, userdata, rc):
        print("Disconnected from MQTT broker")
        self.connected = False
    
    def on_publish(self, client, userdata, mid):
        print(f"Configuration message published (message id: {mid})")
    
    def on_log(self, client, userdata, level, buf):
        print(f"MQTT Log: {buf}")
    
    def connect(self):
        try:
            print(f"Connecting to MQTT broker at {MQTT_BROKER}:{MQTT_PORT} with SSL...")
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.client.loop_start()
            
            # Wait for connection
            timeout = 10
            while not self.connected and timeout > 0:
                time.sleep(1)
                timeout -= 1
            
            if not self.connected:
                print("Failed to connect within timeout period")
                return False
            
            return True
            
        except Exception as e:
            print(f"Connection error: {e}")
            return False
    
    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()
    
    def send_config(self, config_data):
        if not self.connected:
            print("Not connected to MQTT broker")
            return False
        
        try:
            # Convert configuration to JSON
            json_config = json.dumps(config_data, indent=2)
            print(f"Sending configuration:\\n{json_config}")
            
            # Publish the configuration
            result = self.client.publish(MQTT_TOPIC, json_config, qos=2, retain=True)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                print("Configuration sent successfully")
                return True
            else:
                print(f"Failed to send configuration. Error code: {result.rc}")
                return False
                
        except Exception as e:
            print(f"Error sending configuration: {e}")
            return False

def main():
    print("MQTT SSL Configuration Sender")
    print("=" * 40)
    
    # Create MQTT client
    sender = MQTTSSLConfigSender()
    
    # Connect to broker
    if not sender.connect():
        print("Failed to connect to MQTT broker")
        sys.exit(1)
    
    try:
        # Send configuration for all ESP32 devices
        print(f"Sending configuration for {len(ESP32_CONFIGS)} ESP32 device(s)...")
        
        if sender.send_config(ESP32_CONFIGS):
            print("Configuration sent successfully!")
            
            # Wait a bit to ensure message is delivered
            time.sleep(2)
            
            print("\\nConfiguration details:")
            for mac, config in ESP32_CONFIGS.items():
                print(f"  ESP32 MAC: {mac}")
                print(f"    Topic: {config['mqtt_topic']}")
                print(f"    Player: {config['player']}")
                print(f"    Exercise: {config['exercise_type']}")
                print(f"    Hard Iron: {config['hard_iron']}")
                print(f"    Soft Iron: {config['soft_iron']}")
                print()
        else:
            print("Failed to send configuration")
            
    except KeyboardInterrupt:
        print("\\nInterrupted by user")
    
    finally:
        sender.disconnect()
        print("Disconnected from MQTT broker")

if __name__ == "__main__":
    main()

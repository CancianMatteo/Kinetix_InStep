 #!/usr/bin/env python3
"""
Test script for MQTT SSL connection
This script tests the SSL connection to the MQTT broker without sending configuration
"""

import paho.mqtt.client as mqtt
import ssl
import time
import sys

MQTT_BROKER = "192.168.137.212"  # Replace with your broker IP
MQTT_PORT = 8883                 # SSL port

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("✓ Successfully connected to MQTT broker with SSL")
        client.publish("test/connection", "SSL connection test", qos=1)
    else:
        print(f"✗ Failed to connect. Error code: {rc}")

def on_publish(client, userdata, mid):
    print("✓ Test message published successfully")

def on_log(client, userdata, level, buf):
    print(f"MQTT Log: {buf}")

def test_ssl_connection():
    print("Testing MQTT SSL Connection...")
    print("=" * 40)
    
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_publish = on_publish
    client.on_log = on_log
    
    # Setup SSL
    context = ssl.create_default_context(ssl.Purpose.SERVER_AUTH)
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE
    client.tls_set_context(context)
    
    try:
        print(f"Connecting to {MQTT_BROKER}:{MQTT_PORT} with SSL...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
        
        # Wait for connection and publish
        time.sleep(3)
        
        client.loop_stop()
        client.disconnect()
        
        print("Test completed successfully!")
        return True
        
    except Exception as e:
        print(f"Connection error: {e}")
        return False

if __name__ == "__main__":
    success = test_ssl_connection()
    sys.exit(0 if success else 1)

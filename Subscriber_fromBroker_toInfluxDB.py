import json
import paho.mqtt.client as mqtt
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
import signal

# InfluxDB config
INFLUX_URL = "192.168.137.4:8086"
INFLUX_TOKEN = "your-token-here"
INFLUX_ORG = "org"
INFLUX_BUCKET = "esp32"

# MQTT config
MQTT_BROKER = "192.168.137.4"
MQTT_TOPIC = "esp32/imugps"
MQTT_QoS = 2

# InfluxDB client
influx_client = InfluxDBClient(
    url=INFLUX_URL,
    token=INFLUX_TOKEN,
    org=INFLUX_ORG
)
write_api = influx_client.write_api(write_options=SYNCHRONOUS)

def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        print(f"\nFailed to connect: {reason_code}. loop_forever() will retry connection")
    else:
        # we should always subscribe from on_connect callback to be sure our subscription is persisted across reconnections.
        client.subscribe(MQTT_TOPIC,MQTT_QoS)

def signal_handler(sig, frame):
    print('\nCtrl+C detected! Disconnecting...')
    mqtt_client.unsubscribe(MQTT_TOPIC)
    mqtt_client.disconnect()

signal.signal(signal.SIGINT, signal_handler)

def on_subscribe(client, userdata, mid, reason_code_list, properties):
    # Since we subscribed only for a single channel, reason_code_list contains a single entry
    if reason_code_list[0].is_failure:
        print(f"\nBroker rejected you subscription: {reason_code_list[0]}")
    else:
        print(f"\nBroker granted the following QoS: {reason_code_list[0].value}")

def on_unsubscribe(client, userdata, mid, reason_code_list, properties):
    # The reason_code_list is only present in MQTTv5, in MQTTv3 it will always be empty
    if len(reason_code_list) == 0 or not reason_code_list[0].is_failure:
        print("\nSuccessfully unsubscribed!")
    else:
        print(f"\nBroker error: {reason_code_list[0]}")
    client.disconnect()

def on_message(client, userdata, msg):
    try:
        # Decode and parse the incoming message
        data = json.loads(msg.payload.decode())
        print("📥 Received:", data)

        imu_data = data["imu"]
        gps_data = data["gps"]

        # Create a point and write it to InfluxDB
        point = Point("imu_icm_data") \
            .tag("source", "esp32") \
            .field("ax", imu_data[0]["ax"]) \
            .field("ay", imu_data[0]["ay"]) \
            .field("az", imu_data[0]["az"]) \
            .field("gx", imu_data[0]["gx"]) \
            .field("gy", imu_data[0]["gy"]) \
            .field("gz", imu_data[0]["gz"]) \
            .field("mx", imu_data[0]["mx"]) \
            .field("my", imu_data[0]["my"]) \
            .field("mz", imu_data[0]["mz"])
        write_api.write(bucket=INFLUX_BUCKET, record=point)
        print("✅ Written to InfluxDB")

    except Exception as e:
        print("❌ Error:", e)

# Set up the MQTT client
mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqtt_client.enable_logger()
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, 1883, 300)
mqtt_client.subscribe(MQTT_TOPIC)

# Start the MQTT loop
mqtt_client.loop_forever()

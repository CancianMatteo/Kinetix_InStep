import json
from paho.mqtt.client import Client
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

# InfluxDB config
INFLUX_URL = "192.168.137.4:8086"
INFLUX_TOKEN = "your-token-here"
INFLUX_ORG = "org"
INFLUX_BUCKET = "esp32"

# MQTT config
MQTT_BROKER = "192.168.137.4"
MQTT_TOPIC = "esp32/imugps"

# InfluxDB client
influx_client = InfluxDBClient(
    url=INFLUX_URL,
    token=INFLUX_TOKEN,
    org=INFLUX_ORG
)
write_api = influx_client.write_api(write_options=SYNCHRONOUS)

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
mqtt_client = Client()
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, 1883, 60)
mqtt_client.subscribe(MQTT_TOPIC)

# Start the MQTT loop
mqtt_client.loop_forever()

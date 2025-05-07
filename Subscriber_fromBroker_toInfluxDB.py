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
MQTT_TOPIC = "esp32/example"

# InfluxDB client
influx_client = InfluxDBClient(
    url=INFLUX_URL,
    token=INFLUX_TOKEN,
    org=INFLUX_ORG
)
write_api = influx_client.write_api(write_options=SYNCHRONOUS)

def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        print("📥 Received:", data)

        point = Point("data") \
            .tag("type", "example") \
            .field("temp", data["temperature"]) \
            .field("hum", data["humidity"])
        write_api.write(bucket=INFLUX_BUCKET, record=point)
        print("✅ Written to InfluxDB")

    except Exception as e:
        print("❌ Error:", e)

mqtt_client = Client()
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, 1883, 60)
mqtt_client.subscribe(MQTT_TOPIC)
mqtt_client.loop_forever()

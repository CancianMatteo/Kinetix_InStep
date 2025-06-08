import json
import csv
from paho.mqtt.client import Client
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

# InfluxDB config
INFLUX_URL = "192.168.137.212:8086"
INFLUX_TOKEN = "jzTVtFCrrRYGdMvdjXdorOreF-PplWoDiH-3b7XVWuhqNX23duEW7Cm5YfIKzvmLpXFX1Zs7jdWzMYevtyoFFw=="
INFLUX_ORG = "uniud"
INFLUX_BUCKET = "calcio_motta"

# MQTT config
MQTT_BROKER = "192.168.137.212"
MQTT_TOPIC = "calcio/prova"

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

        # Extract IMU arrays from the data
        imu_data = data["imu"]
        ax = imu_data["ax"]
        ay = imu_data["ay"]
        az = imu_data["az"]
        gx = imu_data["gx"]
        gy = imu_data["gy"]
        gz = imu_data["gz"]
        mx = imu_data["mx"]
        my = imu_data["my"]
        mz = imu_data["mz"]

        # Write to InfluxDB
        points = []
        for i in range(len(ax)):
            point = Point("imu_data") \
                .tag("source", data["source"]) \
                .tag("player", data["player"]) \
                .tag("exercise_type", data["exercise_type"]) \
                .field("ax", ax[i]) \
                .field("ay", ay[i]) \
                .field("az", az[i]) \
                .field("gx", gx[i]) \
                .field("gy", gy[i]) \
                .field("gz", gz[i]) \
                .field("mx", mx[i]) \
                .field("my", my[i]) \
                .field("mz", mz[i])
            points.append(point)

        write_api.write(bucket=INFLUX_BUCKET, record=points)
        print(f"✅ Written {len(points)} points to InfluxDB")

        # Log to CSV
        csv_filename = f"{data['player']}_{data['exercise_type']}_{data['source']}.csv"
        with open(csv_filename, mode='a', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            for i in range(len(ax)):
                csv_writer.writerow([ax[i], ay[i], az[i], gx[i], gy[i], gz[i], mx[i], my[i], mz[i]])
        print(f"✅ Logged IMU data to {csv_filename}")

    except Exception as e:
        print("❌ Error:", e)

# Set up the MQTT client
mqtt_client = Client()
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, 1883, 60)
mqtt_client.subscribe(MQTT_TOPIC)

# Start the MQTT loop
mqtt_client.loop_forever()
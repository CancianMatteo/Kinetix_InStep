import json
import csv
import os
from datetime import datetime, timedelta
from paho.mqtt.client import Client
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# InfluxDB config
INFLUX_URL = "192.168.137.212:8086"
INFLUX_TOKEN = os.getenv("INFLUX_TOKEN")
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

path = "/Users/matteocancian/Documents/UNIUD/Tirocinio/Kinetix_InStep/read-IMU_WiFiPubMQTT/csv_files/"

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
        print(f"✅ Written {len(points)} points to InfluxDB {INFLUX_BUCKET}")

        # Parse the current timestamp
        current_time = datetime.strptime(data['fromTime'], "%Y-%m-%dT%H-%M-%S")
        
        # Base filename without timestamp
        base_name = f"{data['player']}_{data['exercise_type']}_{data['source']}"
        
        # Look for existing files from the last minute
        matching_file = None
        if current_time:
            one_minute_ago = current_time - timedelta(minutes=1)
            
            for file in os.listdir(path):
                if file.startswith(base_name) and file.endswith('.csv'):
                    # Extract timestamp from filename
                    timestamp_part = file.replace(f"{base_name}_", "").replace(".csv", "")
                    file_time = datetime.strptime(timestamp_part, "%Y-%m-%dT%H-%M-%S")
                    
                    # Check if file timestamp is within 1 minute
                    if file_time and file_time >= one_minute_ago and file_time <= current_time:
                        matching_file = file
                        break
        
        # Use existing file or create new one
        if matching_file:
            csv_filename = matching_file
            print(f"📎 Appending to existing file: {csv_filename}")
        else:
            csv_filename = f"{base_name}_{data['fromTime']}.csv"
            print(f"📄 Creating new file: {csv_filename}")

        # Log to CSV
        with open(path+csv_filename, mode='a', newline='') as csv_file:
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
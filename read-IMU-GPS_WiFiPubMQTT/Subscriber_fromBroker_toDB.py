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

# Path for CSV files
path = "/Users/matteocancian/Documents/UNIUD/Tirocinio/Kinetix_InStep/read-IMU-GPS_WiFiPubMQTT/csv_files/"
os.makedirs(path, exist_ok=True)  # Create directory if it doesn't exist

def on_message(client, userdata, msg):
    try:
        # Decode and parse the incoming message
        data = json.loads(msg.payload.decode())
        print("📥 Received message from:", data["source"])
        
        # Create base filename with timestamp
        current_time = datetime.strptime(data['fromTime'], "%Y-%m-%dT%H-%M-%S")
        base_name = f"{data['player']}_{data['exercise_type']}_{data['source']}"
        
        # Check for existing files from the last minute
        matching_file = find_matching_file(base_name, current_time)
        
        # Process IMU data
        if "imu" in data:
            process_imu_data(data, matching_file, base_name)
        
        # Process GPS data if available
        if "gps" in data:
            process_gps_data(data, matching_file, base_name)

    except Exception as e:
        print("❌ Error:", e)
        import traceback
        traceback.print_exc()

def find_matching_file(base_name, current_time):
    # Look for existing files from the last minute
    matching_file = None
    if current_time:
        one_minute_ago = current_time - timedelta(minutes=1)
        
        for file in os.listdir(path):
            if file.startswith(base_name) and file.endswith('.csv'):
                # Extract timestamp from filename
                timestamp_part = file.replace(f"{base_name}_", "").replace(".csv", "")
                try:
                    file_time = datetime.strptime(timestamp_part, "%Y-%m-%dT%H-%M-%S")
                    
                    # Check if file timestamp is within 1 minute
                    if file_time and file_time >= one_minute_ago and file_time <= current_time:
                        matching_file = file
                        break
                except ValueError:
                    continue  # Skip files with invalid timestamps
    
    return matching_file

def process_imu_data(data, matching_file, base_name):
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
    print(f"✅ Written {len(points)} IMU points to InfluxDB {INFLUX_BUCKET}")

    # Log IMU data to CSV
    csv_filename = f"{base_name}_{data['fromTime']}_imu.csv" if not matching_file else matching_file.replace('.csv', '_imu.csv')
    
    # Check if file exists to determine whether to write headers
    file_exists = os.path.isfile(path + csv_filename)
    
    with open(path + csv_filename, mode='a', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        
        # Write header if new file
        if not file_exists:
            csv_writer.writerow(['ax', 'ay', 'az', 'gx', 'gy', 'gz', 'mx', 'my', 'mz'])
            
        # Write data rows
        for i in range(len(ax)):
            csv_writer.writerow([ax[i], ay[i], az[i], gx[i], gy[i], gz[i], mx[i], my[i], mz[i]])
    
    print(f"✅ Logged IMU data to {csv_filename}")

def process_gps_data(data, matching_file, base_name):
    # Extract GPS arrays from the data
    gps_data = data["gps"]
    time_values = gps_data["time"]
    lat = gps_data["lat"]
    lng = gps_data["lng"]
    spd = gps_data["spd"]

    # Write to InfluxDB
    points = []
    for i in range(len(lat)):
        point = Point("gps_data") \
            .tag("source", data["source"]) \
            .tag("player", data["player"]) \
            .tag("exercise_type", data["exercise_type"]) \
            .field("lat", lat[i]) \
            .field("lng", lng[i]) \
            .field("spd", spd[i]) \
            .time(time_values[i] if i < len(time_values) else None)  # Use time if available
        points.append(point)

    write_api.write(bucket=INFLUX_BUCKET, record=points)
    print(f"✅ Written {len(points)} GPS points to InfluxDB {INFLUX_BUCKET}")

    # Log GPS data to separate CSV file
    csv_filename = f"{base_name}_{data['fromTime']}_gps.csv" if not matching_file else matching_file.replace('.csv', '_gps.csv')
    
    # Check if file exists to determine whether to write headers
    file_exists = os.path.isfile(path + csv_filename)
    
    with open(path + csv_filename, mode='a', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        
        # Write header if new file
        if not file_exists:
            csv_writer.writerow(['time', 'lat', 'lng', 'spd'])
            
        # Write data rows
        for i in range(len(lat)):
            time_val = time_values[i] if i < len(time_values) else ""
            csv_writer.writerow([time_val, lat[i], lng[i], spd[i]])
    
    print(f"✅ Logged GPS data to {csv_filename}")

# Set up the MQTT client
mqtt_client = Client()
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, 1883, 60)
mqtt_client.subscribe(MQTT_TOPIC)

print(f"🔌 Connected to MQTT broker at {MQTT_BROKER}")
print(f"🎧 Listening on topic: {MQTT_TOPIC}")
print(f"📊 Writing data to InfluxDB bucket: {INFLUX_BUCKET}")
print(f"📁 Saving CSV files to: {path}")

# Start the MQTT loop
mqtt_client.loop_forever()
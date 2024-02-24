import socket
import math
import time
import json
from pymavlink import mavutil


def send_fake_gps(mavlink_connection, lat, lon, alt):
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)

    mavlink_connection.mav.gps_input_send(
        0,  # Timestamp (micros since boot or Unix epoch)
        0,  # ID of the GPS for multiple GPS inputs
        # Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).
        # All other fields must be provided.
        0,
        0,  # GPS time (milliseconds from start of GPS week)
        0,  # GPS week number
        3,  # 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
        lat_int,  # Latitude (WGS84), in degrees * 1E7
        lon_int,  # Longitude (WGS84), in degrees * 1E7
        alt,  # Altitude (AMSL, not WGS84), in m (positive for up)
        1,  # GPS HDOP horizontal dilution of position in m
        1,  # GPS VDOP vertical dilution of position in m
        0,  # GPS velocity in m/s in NORTH direction in earth-fixed NED frame
        0,  # GPS velocity in m/s in EAST direction in earth-fixed NED frame
        0,  # GPS velocity in m/s in DOWN direction in earth-fixed NED frame
        0,  # GPS speed accuracy in m/s
        0,  # GPS horizontal accuracy in m
        0,  # GPS vertical accuracy in m
        16  # Number of satellites visible
        # mavlink_connection.mav.gps_input_send(
        #   0, 0, mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_ALT | mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_HDOP | mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VDOP,
        #  lat * 1e7, lon * 1e7, alt, 10, 10, 0, 0, 0, 0, 10, 3

    )


# Your existing functions (convert_to_gps, send_fake_gps, setup_mavlink_connection) remain the same.
def setup_mavlink_connection(port, baudrate):
    return mavutil.mavlink_connection(port, baudrate=baudrate)


host = '172.20.10.3'
port = 56789

# Creating a UDP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((host, port))

print(f"Listening on {host}:{port}...")

mavlink_conn = setup_mavlink_connection('/dev/serial0', 57600)

while True:

    data, addr = server_socket.recvfrom(1024)  # buffer size is 1024 bytes
    print(f"Received data from {addr}")

    received_data = data.decode('utf-8')
    print("Received data:", received_data)

    try:
        # parts = received_data.split(", ")
        # latitude_part, longitude_part, altitude_part = parts
        #
        # latitude = float(latitude_part.split(":")[1].strip())
        # longitude = float(longitude_part.split(":")[1].strip())
        # altitude = float(altitude_part.split(":")[1].strip())
        received_data_json = json.loads(received_data)
        latitude = received_data_json.get("latitude")
        longitude = received_data_json.get("longitude")
        altitude = received_data_json.get("altitude")


        # lat, lon, alt = convert_to_gps(x, y, z)
        print("latitude:", latitude)
        print("longitude:", longitude)
        print("altitude:", altitude)
        mavlink_conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        print("[INFO]Heartbeat sent")
        send_fake_gps(mavlink_conn, latitude, longitude, altitude)
        # time.sleep(1)
        # x, y, z = [float(val.split(':')[1]) for val in received_data.split(',')]
        # lat, lon, alt = convert_to_gps(x, y, z)
        # send_fake_gps(mavlink_conn, lat, lon, alt)
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON data: {e}")

    except Exception as e:
        print(f"Error parsing data: {e}")

    # Your existing data processing logic remains the same.

    # Send response back to client, if needed
    # server_socket.sendto(response_data.encode('utf-8'), addr)

# server_socket.close()
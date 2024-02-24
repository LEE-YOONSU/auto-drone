import socket
import math
import time
from pymavlink import mavutil


def convert_to_gps(mocap_x, mocap_y, mocap_z, base_latitude=36.1706754, base_longitude=128.4684821):
    latitude_per_meter = 9e-06
    longitude_per_meter = 1.1e-05

    latitude = base_latitude + (mocap_x * latitude_per_meter)
    longitude = base_longitude + (-mocap_y * longitude_per_meter)
    altitude = mocap_z - 0.04
    print("latitude :", int(latitude * 1e7))
    print("longitude :", int(longitude * 1e7))
    print("altitude:", altitude)
    return latitude, longitude, altitude


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


host = '172.20.10.2'
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
        pos_data, rot_data = received_data.split("), ")

        pos_values = pos_data.split(":")[1].strip("()").split(", ")
        x, y, z = [float(val) for val in pos_values]

        rot_data = rot_data.strip("()\n")
        rot_values = rot_data.split(":")[1].strip("()").split(", ")
        roll, pitch, yaw = [float(val) for val in rot_values]

        lat, lon, alt = convert_to_gps(x, y, z)
        print("lat:", lat)
        print("lon:", lon)
        print("alt:", alt)
        mavlink_conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        print("[INFO]Heartbeat sent")
        send_fake_gps(mavlink_conn, lat, lon, alt)
        # time.sleep(1)
        # x, y, z = [float(val.split(':')[1]) for val in received_data.split(',')]
        # lat, lon, alt = convert_to_gps(x, y, z)
        # send_fake_gps(mavlink_conn, lat, lon, alt)
    except Exception as e:
        print(f"Error parsing data: {e}")

    # Your existing data processing logic remains the same.

    # Send response back to client, if needed
    # server_socket.sendto(response_data.encode('utf-8'), addr)

# server_socket.close()

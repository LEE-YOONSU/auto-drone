import socket
import math
import time
import json
from pymavlink import mavutil
import threading


# Your existing functions (convert_to_gps, send_fake_gps, setup_mavlink_connection) remain the same.
def setup_mavlink_connection(port, baudrate):
    return mavutil.mavlink_connection(port, baudrate=baudrate)


host = '192.168.0.106'
port = 56789

time_usec = int(time.time() * 1e6)

# Creating a UDP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((host, port))

print(f"Listening on {host}:{port}...")

mavlink_conn = setup_mavlink_connection('/dev/serial0', 57600)

origin_latitude = 36.1706754
origin_longitude = 128.4684821
origin_altitude = 101000  # 원점의 고도

def send_gps(mavlink_conn, origin_latitude, origin_longitude, origin_altitude, time_usec):
    mavlink_conn.mav.set_gps_global_origin_send(
        mavlink_conn.target_system,
        origin_latitude,
        origin_longitude,
        origin_altitude,
        time_usec
    )
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
        0,  # 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
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
    )


def receive_gps_and_send_mavlink():
    while True:

        data, addr = server_socket.recvfrom(1024)  # buffer size is 1024 bytes
        print(f"Received data from {addr}")

        received_data = data.decode('utf-8')
        print("Received data:", received_data)

        try:
            received_data_json = json.loads(received_data)
            # latitude = received_data_json.get("latitude")
            # longitude = received_data_json.get("longitude")
            # altitude = received_data_json.get("altitude")
            x = received_data_json.get("x")
            y = received_data_json.get("y")
            z = received_data_json.get("z")
            qx = received_data_json.get("qx")
            qy = received_data_json.get("qy")
            qz = received_data_json.get("qz")
            qw = received_data_json.get("qw")

            q = [qw, qx, qy, qz]

            mavlink_conn.mav.att_pos_mocap_send(
                time_usec,
                q,
                x, y, z
            )

            # 목표 위치로 이동하는 명령 전송
            mavlink_conn.mav.set_position_target_local_ned_send(
                time_usec,
                mavlink_conn.target_system,
                mavlink_conn.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,  # 비트마스크
                int(origin_latitude * 1e7),  # 위도
                int(origin_longitude * 1e7),  # 경도
                origin_altitude,  # z 위치는 그대로 유지
                0, 0, 0,  # 속도 (미터/초, 사용 안 함)
                0, 0, 0,  # 가속도 (미터/초^2, 사용 안 함)
                0, 0  # yaw, yaw_rate (사용 안 함)
            )

            mavlink_conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            print("[INFO]Heartbeat sent")

        except json.JSONDecodeError as e:
            print(f"Error decoding JSON data: {e}")

        except Exception as e:
            print(f"Error parsing data: {e}")

        # Your existing data processing logic remains the same.

        # Send response back to client, if needed
        # server_socket.sendto(response_data.encode('utf-8'), addr)

# Start a thread to receive Mocap data and send MAVLink messages concurrently
# send_fake_gps(mavlink_conn, lat, lon, alt)
send_gps(mavlink_conn, origin_latitude, origin_longitude, origin_altitude, time_usec)
thread_receive_and_send = threading.Thread(target=receive_gps_and_send_mavlink)
thread_receive_and_send.start()

# Join the thread to main thread
thread_receive_and_send.join()

# server_socket.close()
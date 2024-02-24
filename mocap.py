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

lat = int(36.1706754 * 1e7)
lon = int(128.4684821 * 1e7)
alt = 0


def send_gps(mavlink_conn, lat, lon, alt):
    mavlink_conn.mav.set_gps_global_origin_send(
        mavlink_conn.target_system,
        lat,
        lon,
        alt,
        time_usec
    )


def receive_gps_and_send_mavlink():
    while True:

        # time_usec = int(time.time() * 1e6)

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

            # lat, lon, alt = convert_to_gps(x, y, z)
            # print("latitude:", latitude)
            # print("longitude:", longitude)
            # print("altitude:", altitude)
            mavlink_conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            print("[INFO]Heartbeat sent")

            # lat = int(36.1706754 * 1e7)
            # lon = int(128.4684821 * 1e7)
            # alt = 0

            send_gps(mavlink_conn, lat, lon, alt)

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


# def send_mocap_data():
# master = mavutil.mavlink_connection("/dev/serial0", baud=57600)
# while True:
#     x, y, z = 0.0, 0.0, 0.0
#     qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
#     time_usec = int(time.time() * 1e6)
#
#     master.mav.att_pos_mocap_send(
#         time_usec,
#         x, y, z,
#         qw, qx, qy, qz
#     )
#
#     time.sleep(0.1)

# Start a thread to receive Mocap data and send MAVLink messages concurrently
thread_receive_and_send = threading.Thread(target=receive_gps_and_send_mavlink)
thread_receive_and_send.start()

# Join the thread to main thread
thread_receive_and_send.join()

# server_socket.close()

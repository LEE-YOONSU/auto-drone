import socket
import math
import time
import json
from pymavlink import mavutil
import threading


######################## data receive ######################################

# Your existing functions (convert_to_gps, send_fake_gps, setup_mavlink_connection) remain the same.
def setup_mavlink_connection(port, baudrate):
    return mavutil.mavlink_connection(port, baudrate=baudrate)


host = '192.168.0.106'
port = 56789

# Creating a UDP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((host, port))

print(f"Listening on {host}:{port}...")

mavlink_conn = setup_mavlink_connection('/dev/serial0', 57600)

################## drone xy In ned ##############################################

def set_position_xy_target_local_ned(x, y):
    # type_mask (0b0000111111000111) ignores z axis (altitude)
    type_mask = 0b0000111111000111
    time_boot_ms = 0  # The number of milliseconds elapsed since system boot
    target_system = 1  # System ID
    target_component = 1  # Component ID

    # Set current altitude to None, as we are not controlling it here
    # The drone should maintain its current altitude automatically

    mavlink_conn.mav.set_position_target_local_ned_send(
        time_boot_ms,
        target_system,
        target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Use LOCAL_NED frame
        type_mask,
        x, y, 0,  # Position (NED) - Z is ignored
        0, 0, 0,  # Velocity (not used)
        0, 0, 0,  # Acceleration (not used)
        0, 0  # Yaw angle (not used), Yaw rate (not used)
    )
    print(f"Set position XY target local NED: x={x}, y={y}")

def receive_gps_and_send_mavlink():
    while True:

        time_usec = int(time.time() * 1e6)

        data, addr = server_socket.recvfrom(1024)  # buffer size is 1024 bytes
        print(f"Received data from {addr}")

        received_data = data.decode('utf-8')
        print("Received data:", received_data)

        try:

            received_data_json = json.loads(received_data)
            x = received_data_json.get("x")
            y = received_data_json.get("z")
            z = -received_data_json.get("y")
            qx = received_data_json.get("qx")
            qy = received_data_json.get("qy")
            qz = received_data_json.get("qz")
            qw = received_data_json.get("qw")

            q = [qw, qx, qz, -qy]

            mavlink_conn.mav.att_pos_mocap_send(
                time_usec,
                q,
                x, y, z
            )

            set_position_xy_target_local_ned(x, y)
            mavlink_conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            print("[INFO]Heartbeat sent")


        except json.JSONDecodeError as e:
            print(f"Error decoding JSON data: {e}")

        except Exception as e:
            print(f"Error parsing data: {e}")

######################### main ################################################################


thread_receive_and_send = threading.Thread(target=receive_gps_and_send_mavlink)
thread_receive_and_send.start()

# Join the thread to main thread
thread_receive_and_send.join()

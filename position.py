import socket
import math
import time
import json
from pymavlink import mavutil
import threading


# Setup mavlink connection
def setup_mavlink_connection(port, baudrate):
    return mavutil.mavlink_connection(port, baudrate=baudrate)


host = '192.168.0.106'
port = 56789

# Creating a UDP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((host, port))

print(f"Listening on {host}:{port}...")

mavlink_conn = setup_mavlink_connection('/dev/serial0', 57600)

# Global variables for origin coordinates and altitude
origin_x = 0
origin_y = 0
origin_z = 0
target_altitude = 0  # Initialize target altitude


# Function to set origin coordinates
def set_origin_coordinates(x, y, z):
    global origin_x, origin_y, origin_z
    origin_x = x
    origin_y = y
    origin_z = z


# Function to calculate distance from origin
# 유클리드 거리를 계산하는 함수로, 세 개의 좌표 간의 직선 거리를 계산한다.
def calculate_distance_from_origin(current_x, current_y, current_z):
    distance = math.sqrt((current_x - origin_x) ** 2 + (current_y - origin_y) ** 2 + (current_z - origin_z) ** 2)
    return distance


# Function to set position target in local NED frame
def set_position_target_local_ned(x, y, z):
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
        x, y, z,  # Position (NED)
        0, 0, 0,  # Velocity (not used)
        0, 0, 0,  # Acceleration (not used)
        0, 0  # Yaw angle (not used), Yaw rate (not used)
    )
    print(f"Set position target local NED: x={x}, y={y}, z={z}")


# Function to receive GPS data and send mavlink messages
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

            set_position_target_local_ned(x, y, target_altitude)
            mavlink_conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            print("[INFO]Heartbeat sent")

            # Calculate distance from origin
            distance_from_origin = calculate_distance_from_origin(x, y, z)
            # Define a threshold distance for hovering
            # 드론이 안정적으로 호버링하기 위한 임계값을 설정한다. 이 임계값은 드론이 원점에서 멀어지면 다시 원점으로 이동할 거리의 기준이 된다.
            hover_threshold = 0.5

            if distance_from_origin > hover_threshold:
                # Drone is outside hover threshold, navigate back to origin
                set_position_target_local_ned(origin_x, origin_y, target_altitude)

        except json.JSONDecodeError as e:
            print(f"Error decoding JSON data: {e}")

        except Exception as e:
            print(f"Error parsing data: {e}")


# Main function to start the thread for receiving GPS data and sending mavlink messages
def main():
    thread_receive_and_send = threading.Thread(target=receive_gps_and_send_mavlink)
    thread_receive_and_send.start()
    # Join the thread to main thread
    thread_receive_and_send.join()


if __name__ == "__main__":
    main()

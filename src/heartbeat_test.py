import socket
import time
import pymavlink.mavutil as mavutil

# Define the IP address and port
IP_ADDRESS = "192.168.0.24"
PORT = 14550

# Define the connection timeout in seconds
CONNECTION_TIMEOUT = 5
HEARTBEAT_TIMEOUT = 10

# Define the methods to try for establishing the connection
methods = [
    lambda: mavutil.mavlink_connection(f"udpbcast://{IP_ADDRESS}:{PORT}", timeout=CONNECTION_TIMEOUT),
    lambda: mavutil.mavlink_connection(f"udpbcast://{IP_ADDRESS}:{PORT}", source_system=1, timeout=CONNECTION_TIMEOUT),
    lambda: mavutil.mavlink_connection(f"udpbcast://{IP_ADDRESS}:{PORT}", source_system=1, dialect="ardupilotmega", timeout=CONNECTION_TIMEOUT),
    lambda: mavutil.mavlink_connection(f"udpin://{IP_ADDRESS}:{PORT}", timeout=CONNECTION_TIMEOUT),
    lambda: mavutil.mavlink_connection(f"udpin://{IP_ADDRESS}:{PORT}", source_system=1, timeout=CONNECTION_TIMEOUT),
    lambda: mavutil.mavlink_connection(f"udpin://{IP_ADDRESS}:{PORT}", source_system=1, dialect="ardupilotmega", timeout=CONNECTION_TIMEOUT),
    lambda: mavutil.mavlink_connection(f"udp://{IP_ADDRESS}:{PORT}", timeout=CONNECTION_TIMEOUT),
    lambda: mavutil.mavlink_connection(f"udp://{IP_ADDRESS}:{PORT}", source_system=1, timeout=CONNECTION_TIMEOUT),
    lambda: mavutil.mavlink_connection(f"udp://{IP_ADDRESS}:{PORT}", source_system=1, dialect="ardupilotmega", timeout=CONNECTION_TIMEOUT),
    lambda: mavutil.mavlink_connection(f"udp://{IP_ADDRESS}:{PORT}", source_component=1, timeout=CONNECTION_TIMEOUT),
    lambda: mavutil.mavlink_connection(f"udp://{IP_ADDRESS}:{PORT}", source_component=1, dialect="ardupilotmega", timeout=CONNECTION_TIMEOUT),
]

# Function to establish the connection
def establish_connection():
    print(f"Attempting to establish connection to {IP_ADDRESS}:{PORT}")
    for i, method in enumerate(methods, start=1):
        try:
            print(f"Trying method {i}/{len(methods)}: {method.__name__}")
            conn = method()
            print(f"Waiting for heartbeat (timeout: {HEARTBEAT_TIMEOUT} seconds)...")
            conn.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT)
            print(f"Heartbeat from system ID: {conn.target_system}")
            return conn
        except Exception as e:
            print(f"Failed to establish connection using method {i}: {e}")
            continue
    return None

# Establish the connection
connection = establish_connection()

# If a connection is established, send and receive messages
if connection:
    print("Connection established successfully!")
    while True:
        msg = connection.recv_match(blocking=True)
        if not msg:
            continue
        else:
            print(f"Received message: {msg}")
            # Example: send a heartbeat message
            connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                          mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                          0, 0, 0)

            # Request GPS data every 2 seconds
            if msg.get_type() == 'HEARTBEAT':
                connection.mav.send(mavutil.mavlink.MAVLink_gps_raw_int_get_message(
                    mavutil.mavlink.MAVLink_message('GPS_RAW_INT', 0, 0, 0, 0, 0)))
                time.sleep(2)
else:
    print("Failed to establish a connection.")


import logging
import struct
import threading
import subprocess
import time
import math
import pandas as pd
import os
import src.params as params
from src.params import Params
from pymavlink import mavutil

from enum import Enum
class State(Enum):
    IDLE = 0
    ARMED = 1
    TRIGGERED = 2

class Mission(Enum):
    NONE = 0
    DRONE_SHOW_FROM_CSV = 1
    SMART_SWARM = 2


params = params.Params()
serial_mavlink = True
Radio_Serial_Baudrate = 57600
Pymavlink_Port = 13540



telem_struct_fmt = '=BHHBBIddddddddBIB'
command_struct_fmt = '=B B B B B I B'

telem_packet_size = struct.calcsize(telem_struct_fmt)
command_packet_size = struct.calcsize(command_struct_fmt)

# Setup logger
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# Single Drone
single_drone = True  # Set this to True for single drone connection

# Read the config file
config_df = pd.read_csv('config.csv')

# Drones list
drones = []

if single_drone:
    # Read the hardware ID from the '.hwID' file
    hw_id = 24  # Extract the hw_id

    # Find the configuration for the drone in the 'config.csv' file
    drone_config = config_df.loc[config_df['hw_id'] == hw_id].iloc[0]
    drones = [drone_config]
else:
    # Add all drones from config file to drones list
    drones = [drone for _, drone in config_df.iterrows()]


# Function to send commands
def send_command(trigger_time, master, hw_id, pos_id, mission, state):
    """
    This function prepares and sends commands.

    :param n: An integer used to compute trigger_time.
    :param master: The pymavlink master through which data will be sent.
    :param coordinator_ip: The IP address of the coordinator.
    :param debug_port: The port used for sending data.
    :param hw_id: The hardware ID.
    :param pos_id: The position ID.
    :param mission: The mission ID.
    :param state: The state value.
    """
    try:
        # Prepare the command data
        header = 55  # Constant
        terminator = 66  # Constant

        # Encode the data
        data = struct.pack(command_struct_fmt, header, hw_id, pos_id, mission, state, trigger_time, terminator)

        # Send the command data
        master.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO,
            data.encode('utf-8')[:50]
        )
        logger.info(f"Sent {len(data)} byte command: Header={header}, HW_ID={hw_id}, Pos_ID={pos_id}, Mission={mission}, State={state}, Trigger Time={trigger_time}, Terminator={terminator}")

    except (OSError, struct.error) as e:
        # If there is an OSError or an error in packing the data, log the error
        logger.error(f"An error occurred: {e}")


def handle_telemetry(keep_running, print_telemetry, master):
    """
    This function continuously receives and handles telemetry data.

    :param keep_running: A control flag for the while loop. 
                         When it's False, the function stops receiving data.
    :param print_telemetry: A flag to control if the telemetry data should be printed.
    :param master: The pymavlink master from which data will be received.
    """
    while keep_running[0]:
        try:
            # Receive SAR Comms -> Target Localization
            msg = master.recv_match()
            if msg and msg.get_type() == 'STATUSTEXT':
                print(f"ID: {msg.get_srcSystem} RSSI: {msg.text}")


            # Receive telemetry data -> Agent Localization
            if msg and msg.get_type() == 'UTM_GLOBAL_POSITION':
                print(f"ID: {msg.get_srcSystem()} LAT: {msg.lat} LON: {msg.lon} ALT: {msg.alt}")
                print(f"VE: {msg.vx} VN: {msg.vy} VD: {msg.vz}")

            if msg and msg.get_type() == 'HEARTBEAT':
                print(f"heartbeat msg\n\tID: {msg.get_srcSystem()} MODE: {msg.custom_mode}")

                # # If received data is not of correct size, log the error and continue
                # if len(data) != telem_packet_size:
                #     logger.error(f"Received packet of incorrect size. Expected {telem_packet_size}, got {len(data)}.")
                #     continue

                # # Decode the data
                # telemetry_data = struct.unpack(telem_struct_fmt, data)
                # header, terminator = telemetry_data[0], telemetry_data[-1]

                # # If header or terminator are not as expected, log the error and continue
                # if header != 77 or terminator != 88:
                #     logger.error("Invalid header or terminator received in telemetry data.")
                #     continue

                # # If the print_telemetry flag is True, print the decoded data
                # if print_telemetry[0]:
                #     hw_id, pos_id, state, mission, trigger_time, position_lat, position_long, position_alt, velocity_north, velocity_east, velocity_down, yaw, battery_voltage, follow_mode, telemetry_update_time = telemetry_data[1:-1]
                #     # Debug log with all details
                #     logger.info(f"Received telemetry at {telemetry_update_time}: Header={header}, HW_ID={hw_id}, Pos_ID={pos_id}, State={State(state).name}, Mission={Mission(mission).name}, Trigger Time={trigger_time}, Position Lat={position_lat}, Position Long={position_long}, Position Alt={position_alt:.1f}, Velocity North={velocity_north:.1f}, Velocity East={velocity_east:.1f}, Velocity Down={velocity_down:.1f}, Yaw={yaw:.1f}, Battery Voltage={battery_voltage:.1f}, Follow Mode={follow_mode}, Terminator={terminator}")
                
        except (OSError, struct.error) as e:
            # If there is an OSError or an error in unpacking the data, log the error and break the loop
            logger.error(f"An error occurred: {e}")
            break
        

# Drones threads
drones_threads = []

# This flag indicates if the telemetry threads should keep running.
# We use a list so the changes in the main thread can be seen by the telemetry threads.
keep_running = [True]

def init_mavlink_comms():
    # Init Mavlink Connection
    master = mavutil.mavlink_connection(f'udp:localhost:{params.comms_port}', source_system=any)
    print(f"Comms: Waiting for Heartbeat at udp:localhost:{params.comms_port}")
    master.wait_heartbeat()
    print(f'Comms: Heartbeat from system (system {master.target_system} component {master.target_system})')
    return master

try:
    # Extract variables
    coordinator_ip = drone_config['ip']
    debug_port = int(drone_config['debug_port'])  # Debug port
    gcs_ip = drone_config['gcs_ip']  # GCS IP
    hw_id = drone_config['hw_id']  # Hardware ID
    pos_id = drone_config['pos_id']  # Position ID

    # Log information
    logger.info(f"Drone {hw_id} is listening and sending on IP {coordinator_ip} and port {debug_port}")

        # Pymavlink Connection
    try:
        master = init_mavlink_comms()
    except Exception as e:
        logging.error(f"An error occured Pymavlink Initialize: {e}")

        # This flag controls whether telemetry is printed to the screen. 
        # We use a list so the changes in the main thread can be seen by the telemetry threads.
    print_telemetry = [True]

        # Start the telemetry thread
    telemetry_thread = threading.Thread(target=handle_telemetry, args=(keep_running, print_telemetry, master))
    telemetry_thread.start()

        # Add to the drones_threads
    drones_threads.append((master, telemetry_thread, coordinator_ip, debug_port, hw_id, pos_id))

    # Main loop for command input
    mission = 0
    state = 0
    n = 0

    master.mav.request_data_stream_send(master.target_system, master.target_component, 
                                        mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)
    time.sleep(1)
    while True:
        msg = master.recv_match()
        if msg:
            print(msg)

        # Turn off telemetry printing while sending commands
        # for _, _, _, _, _, _ in drones_threads:
        #     print_telemetry[0] = False
        # # Send command to each drone
        # for master, _, coordinator_ip, debug_port, hw_id, pos_id in drones_threads:
        #     trigger_time = int(time.time()) + int(n)  # Now + n seconds
        #     send_command(trigger_time, master, coordinator_ip, debug_port, hw_id, pos_id, mission, state)
        #     # Turn on telemetry printing after sending commands
        # for _, _, _, _, _, _ in drones_threads:
        #     print_telemetry[0] = True
except (ValueError, OSError, KeyboardInterrupt) as e:
    # Catch any exceptions that occur during the execution
    logger.error(f"An error occurred: {e}")
finally:
    # When KeyboardInterrupt happens or an error occurs, stop the telemetry threads
    keep_running[0] = False

    for master, telemetry_thread, _, _, _, _ in drones_threads:
        # Close the pymavlink connection
        master.close()
        # Join the thread
        telemetry_thread.join()

logger.info("Exiting the application...")


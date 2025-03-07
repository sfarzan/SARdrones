"""
    A class to encapsulate various configuration parameters needed for the application. This includes connection settings,
    timing settings, packet formats and sizes, and URL locations for the configuration files.

    Attributes
    ----------
    config_url : str
        The URL for the configuration file.
    swarm_url : str
        The URL for the swarm file.
    sim_mode : bool
        Whether to use simulation mode or not.
    serial_mavlink : bool
        Whether the Raspberry Pi is connected to Pixhawk using serial or UDP.
    serial_mavlink_port : str
        The default serial port for Raspberry Pi Zero.
    serial_baudrate : int
        The default baud rate for the serial connection.
    sitl_port : int
        The default SITL port.
    gcs_mavlink_port : int
        The port to send Mavlink messages to GCS.
    mavsdk_port : int
        The default MAVSDK port.
    local_mavlink_port : int
        Local Mavlink port.
    shared_gcs_port : bool
        Whether to share the GCS port or not.
    extra_devices : list
        List of extra devices (IP:Port) to route Mavlink.
    sleep_interval : float
        The sleep interval for the main loop in seconds.
    offline_config : bool
        Whether to use offline configuration or not.
    offline_swarm : bool
        Whether to use offline swarm or not.
    default_sitl : bool
        Whether to use the default 14550 port for single drone simulation.
    online_sync_time : bool
        Whether to sync time from Internet Time Servers.
    TELEM_SEND_INTERVAL : int
        Send telemetry data every TELEM_SEND_INTERVAL seconds.
    local_mavlink_refresh_interval : float
        Refresh interval for local Mavlink connection.
    broadcast_mode : bool
        Whether to use broadcast mode or not.
    telem_struct_fmt : str
        Telemetry packet format.
    command_struct_fmt : str
        Command packet format.
    telem_packet_size : int
        Size of telemetry packet.
    command_packet_size : int
        Size of command packet.
    income_packet_check_interval : float
        Interval for checking incoming packets.
    default_GRPC_port : int
        Default GRPC port.
    offboard_follow_update_interval : float
        Offboard follow update interval.
    """

# import struct
import glob
import os
# from enum import Enum

# class State(Enum):
#     IDLE = 0
#     ARMED = 1
#     TRIGGERED = 2
#     SWARM = 3
#     FOLLOW_CSV = 4
#     HOME = 5
#     LAND = 6
#     TAKEOFF = 7

# def read_hw_id():
#     hwid_files = glob.glob('**/*.hwID', recursive=True)
#     # Print each file path and filename
#     for file_path in hwid_files:
#         filename = os.path.basename(file_path)
#         print("File path:", file_path)
#         print("Filename:", filename)
#         hw_id = int(os.path.splitext(filename)[0])  # Get filename without extension
#         print(hw_id)
#         return int(hw_id)
#     return None

# class Params():

#     # Configuration Variables
#     # URLs
#     config_url = 'https://alumsharif.org/download/config.csv'  # URL for the configuration file
#     swarm_url = 'https://alumsharif.org/download/swarm.csv'  # URL for the swarm file

#     # Simulation mode switch
#     sim_mode = False  # Set to True for simulation mode, False for real-life mode

#     # Mavlink Connection
#     serial_mavlink = False  # Set to True if Raspberry Pi is connected to Pixhawk using serial, False for UDP
#     serial_mavlink_port = 'ttyACM0'  # Default serial port for Raspberry Pi Zero
#     serial_baudrate = 57600  # Default baudrate
                
#     # CONFIGURES PORT NUMBERS BASED ON HW_ID IF SIM_MODE IS TRUE    
#     hw_id = read_hw_id()
#     if sim_mode == True:
#         sitl_port = 14540 + hw_id  # SIM SITL port - Used for Recieving Mavlink from SIM
#         mavsdk_port = 14550 + hw_id # SIM MAVSDK port - Recieves forwarded Mavlink - Interface for MAVSDK Commands
#         local_mavlink_port = 12550 + hw_id  # Local Mavlink port - Used to Display Mavlink Data to Terminal
#         grpc_port = 50050 + hw_id # Default GRPC Port - Used for Offboard Control
#     else:
#         sitl_port = 14540  # Default SITL port
#         mavsdk_port = 14550 + hw_id # Default MAVSDK port
#         local_mavlink_port = 12550 + hw_id # Default Local Mavlink port
#         grpc_port = 50050 # Default GRPC Port
#         comms_port = 13540 + hw_id # Port for Pymavlink Status Messages to be Sent (HW_ID, telemetry, RSSI Values)

#     shared_gcs_port = True
#     #extra_devices = [f"127.0.0.1:{local_mavlink_port}", "192.168.189.1:14550"]  # List of extra devices (IP:Port) to route Mavlink
#     extra_devices = [f"127.0.0.1:{local_mavlink_port}"]
#     # Sleep interval for the main loop in seconds
#     sleep_interval = 0.1

#     # Offline configuration switch
#     offline_config = True  # Set to True to use offline configuration
#     offline_swarm = True  # Set to True to use offline swarm

#     # Default SITL port for single drone simulation
#     default_sitl = True  # Set to True to use default 14550 port for single drone simulation

#     # Online time synchronization switch
#     online_sync_time = False  # Set to True to sync time from Internet Time Servers

#     # Telemetry and Communication
#     TELEM_SEND_INTERVAL = 0.5  # Send telemetry data every TELEM_SEND_INTERVAL seconds
#     local_mavlink_refresh_interval = 0.1  # Refresh interval for local Mavlink connection
#     broadcast_mode = True  # Set to True for broadcast mode, False for unicast mode

#     # Packet formats
#     telem_struct_fmt = '=BHHBBIddddddddBIB'  # Telemetry packet format
#     command_struct_fmt = '=B B B B B I B'  # Command packet format

#     # Packet sizes
#     telem_packet_size = struct.calcsize(telem_struct_fmt)  # Size of telemetry packet
#     command_packet_size = struct.calcsize(command_struct_fmt)  # Size of command packet

#     # Interval for checking incoming packets
#     income_packet_check_interval = 0.02



#     # Offboard follow update interval
#     offboard_follow_update_interval = 0.2


#     schedule_mission_frequency = 2
#     follow_setpoint_frequency = 4


#     class Mission(Enum):
#         NONE = 0
#         DRONE_SHOW_FROM_CSV = 1
#         SMART_SWARM = 2
#         TAKE_OFF = 10
#         LAND = 101
#         HOLD = 102
#         TEST = 100

def main():
    print("me")

    # hwid_files = glob.glob('**/*.hwID', recursive=True)
    current_directory = os.getcwd()

    # Get the grandparent directory path
    grandparent_directory = os.path.abspath(os.path.join(current_directory, os.pardir, os.pardir))

    # Search for files with the extension ".hwID" in the grandparent directory recursively
    hwid_files = glob.glob(os.path.join(grandparent_directory, '**/*.hwID'), recursive=True)


    # Print each file path and filename
    for file_path in hwid_files:
        filename = os.path.basename(file_path)
        print("File path:", file_path)
        print("Filename:", filename)
        hw_id = int(os.path.splitext(filename)[0])  # Get filename without extension
        print(hw_id)
        return hw_id
    return None
    

if __name__ == "__main__":
    main()
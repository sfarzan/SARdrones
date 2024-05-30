import argparse
import asyncio
import csv
from mavsdk import System, telemetry
import glob
import os
import subprocess
import logging
from params import Params as params
from pymavlink import mavutil
import psutil  # You may need to install this package
import time
import threading
from concurrent.futures import ThreadPoolExecutor
import sys

# stop_flag = threading.Event()
# executor = ThreadPoolExecutor(max_workers=10)

def read_config(filename='config.csv'):
    print("Reading drone configuration...")
    dronesConfig = []
    with open(filename, newline='') as csvfile:
        reader = csv.reader(csvfile)
        next(reader, None)  # Skip the header
        for row in reader:
            hw_id, pos_id, x, y, ip, mavlink_port, debug_port, gcs_ip = row
            if int(hw_id) == HW_ID:
                print(f"Matching hardware ID found: {hw_id}")
                droneConfig = {
                    'hw_id': hw_id,
                    'udp_port': mavlink_port,
                    'grpc_port': debug_port
                }
                print(f"Drone configuration: {droneConfig}")
                return droneConfig
    print("No matching hardware ID found in the configuration file.")
            
HW_ID = params.hw_id
SIM_MODE = False  # or True based on your setting
GRPC_PORT_BASE = 50050
UDP_PORT_BASE = 14550 + HW_ID

# Function for ensuring GPS fix before drone arm to avoid COMMAND_DENIED error
async def check_gps_fix_and_arm(drone):        
    start_time = asyncio.get_event_loop().time()

    async for telemetry_data in drone.telemetry.gps_info():
        if telemetry_data.fix_type == telemetry.FixType.FIX_3D:
            print("GPS Fix Obtained")
            async for is_armed in drone.telemetry.armed():
                if is_armed:
                    print("The drone is already armed.")
                    break
                else:
                    print("Arming Drone")
                    await drone.action.arm()
                    break
            break
        if asyncio.get_event_loop().time() - start_time > 10:
            print("Timeout waiting for GPS fix")
            break
        await asyncio.sleep(1)


def arm_drone(drone):
    print("Arming the drone...")
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)
    # print("Vehicle armed")
        # Wait for the drone to arm
    ack = drone.recv_match(type='COMMAND_ACK', blocking=True)
    if ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Drone armed successfully.")
    else:
        raise Exception("Arming the drone failed with result: {}".format(ack.result))
    try:
        drone.motors_armed_wait()
        print("Motors armed")
    except Exception as e:
        print(f"Error waiting for motors to arm: {e}")


def disarm_drone(master):
    """
    Disarm the drone.
    """
    print("Disarming the drone...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,  # 1 to arm, 0 to disarm
        0, 0, 0, 0, 0, 0)

    # Wait for the acknowledgment
    ack = master.recv_match(type='COMMAND_ACK', blocking=True)
    if ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Drone disarmed successfully.")
    else:
        raise Exception("Disarming the drone failed with result: {}".format(ack.result))

def guided_mode(master):
    """
    Set the drone to guided mode.
    """
    print("Setting the drone to guided mode...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # Bitmask to use custom_mode
        4,  # Mode 4: Guided mode
        0, 0, 0, 0, 0)
    
    ack = master.recv_match(type='COMMAND_ACK', blocking=True)
    if ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Guided mode set successfully.")
    else:
        raise Exception("Setting the drone to guided mode failed with result: {}".format(ack.result))



    # print("Setting the drone to guided mode...")
    # master.mav.command_long_send(
    #     master.target_system,
    #     master.target_component,
    #     mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    #     0,
    #     216,  # Mode 4: Guided mode
    #     0, 0, 0, 0, 0, 0)

    # # Wait for the acknowledgment
    # ack = master.recv_match(type='COMMAND_ACK', blocking=True)
    # if ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    #     print("Guided mode set successfully.")
    # else:
    #     raise Exception("Setting the drone to guided mode failed with result: {}".format(ack.result))
    
def takeoff(master, altitude=20):
    """
    Command the drone to take off to the specified altitude.
    """
    print(f"Taking off to {altitude} meters...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        altitude)  # Target altitude in meters

    # Wait for the acknowledgment
    ack = master.recv_match(type='COMMAND_ACK', blocking=True)
    if ack.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Takeoff command accepted.")
    else:
        raise Exception("Takeoff command failed with result: {}".format(ack.result))
    
    print("Waiting for takeoff...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg and msg.relative_alt >= altitude * 1000 * 0.95:  # Allow a small tolerance
            print(f"Reached target altitude: {msg.relative_alt / 1000.0} meters")
            break
        time.sleep(0.1)

def land(master):
    """
    Command the drone to land.
    """
    print("Landing...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0)

    # Wait for the acknowledgment
    ack = master.recv_match(type='COMMAND_ACK', blocking=True)
    if ack.command == mavutil.mavlink.MAV_CMD_NAV_LAND and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Land command accepted.")
    else:
        raise Exception("Land command failed with result: {}".format(ack.result))

def hold_position(master):
    """
    Hold the current position and altitude using MAV_CMD_NAV_LOITER_UNLIM.
    """
    # Read current position from the vehicle's global position estimate
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg is not None:
        # Extract current position and altitude
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1e3  # Convert from mm to meters

        # Send loiter command to the vehicle
        master.mav.command_long_send(
            master.target_system,  # target_system
            master.target_component,  # target_component
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,  # command
            0,  # confirmation
            0, 0,  # param1-2 (not used)
            0,  # param3 (Loiter radius, set to 0 for point loiter)
            float('nan'),  # param4 (Yaw angle, NaN to maintain current yaw heading mode)
            int(lat * 1e7),  # param5 (Latitude)
            int(lon * 1e7),  # param6 (Longitude)
            alt)  # param7 (Altitude)

async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return
    
async def perform_action(action, altitude):
    print("Starting to perform action...")
    print(f"SIM_MODE: {SIM_MODE}, GRPC_PORT_BASE: {GRPC_PORT_BASE}, HW_ID: {HW_ID}")

    grpc_port = GRPC_PORT_BASE
    udp_port = UDP_PORT_BASE

    print(f"gRPC Port: {grpc_port}")
    print(f"UDP Port: {udp_port}")
    
    # Start mavsdk_server
    # mavsdk_server = start_mavsdk_server(grpc_port, udp_port)
    try:
        drone = System(sysid=200+params.hw_id)
        await drone.connect(system_address=f"udp://:{params.mavsdk_port}")
        drone_id_param = await drone.param.get_param_int("MAV_SYS_ID")
        while drone_id_param != params.hw_id:
            print(f"wrong id: {drone_id_param} vs {params.hw_id}")
            await drone.connect(system_address=f"udp://:{params.mavsdk_port}")
            # get the system id parameter
            drone_id_param = await drone.param.get_param_int("MAV_SYS_ID")
        print(f"sysid = {drone._sysid} drone_id_param = {drone_id_param}")

        status_text_task = asyncio.ensure_future(print_status_text(drone))

        print("Waiting for drone to connect...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                print(f"-- Connected to drone!")
                break

        print("Waiting for drone to have a global position estimate...")
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
                break
        # Init Mavlink Connection
        # master = mavutil.mavlink_connection(f'udp:localhost:{params.mavsdk_port}', source_system=params.hw_id + 1)
        
        # print(f"Comms: Waiting for Heartbeat at udp:localhost:{params.comms_port}")
        # master.wait_heartbeat()
        # print(f'Comms: Heartbeat from system (system {master.target_system} component {master.target_system})')

    except Exception as e:
        logging.error(f"Error starting pymavlink: {e}")
        return None

    # heartbeat_thread = threading.Thread(target=send_heartbeat, args=(master,))
    # heartbeat_thread.start()
    time.sleep(2)
    # Perform the action
    try:
        if action == "takeoff":
            print("-- Arming")
            await drone.action.arm()

            print("-- Taking off")
            await drone.action.takeoff(altitude)

            await asyncio.sleep(10)

            print("-- Landing")
            await drone.action.land()

            status_text_task.cancel()
            # arm_drone(master)
            # # guided_mode(master)
            # takeoff(master, altitude)
        # elif action == "land":
        #     land(master)
        # elif action == "hold":
        #     hold_position(master)
        # elif action == "test":
        #     arm_drone(master)
        #     await asyncio.sleep(4)
        #     disarm_drone(master)
        else:
            print("Invalid action")
    except Exception as e:
        print(f"ERROR DURING ACTION: {e}")
    finally:
        # heartbeat_thread.join()
        executor.shutdown()
        # master.close()

def send_heartbeat(master):
    while not stop_flag.is_set():
        # print("sending heartbeat...")
        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(1)

if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Perform actions with drones.")
    parser.add_argument('--action', type=str, required=True, help='Action to perform: takeoff, land, hold')
    parser.add_argument('--altitude', type=float, default=20, help='Altitude for takeoff')

    args = parser.parse_args()

    # Run the main event loop
    loop = asyncio.get_event_loop()
    loop.run_until_complete(perform_action(args.action, args.altitude))
    # stop_flag.set()
    sys.exit("Script executed successfully.")


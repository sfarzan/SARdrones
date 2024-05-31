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
from mavsdk.follow_me import (Config, FollowMeError, TargetLocation)

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

async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return
async def fake_hold(drone):
    
    latitude = 47.398039859999997
    longitude = 8.5455725400000002
    async for position in drone.telemetry.position():
        print(f"Position: {position.latitude_deg}, {position.longitude_deg}")
        latitude = position.latitude_deg
        longitude = position.longitude_deg
        await asyncio.sleep(1)
        break

    target = TargetLocation(latitude, longitude - 0.000005, 0, 0, 0, 0)
    print("-- Starting Follow Me Mode")
    await drone.follow_me.start()
    await asyncio.sleep(8)
    print("-- Following Target")
    await drone.follow_me.set_target_location(target)
    await asyncio.sleep(10)
    print("-- Stopping Follow Me Mode")
    await drone.follow_me.stop()
    
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

    # heartbeat_thread = threading.Thread(target=send_heartbeat, args=(drone,))
    # heartbeat_thread.start()
    time.sleep(2)
    # Perform the action
    try:
        if action == "takeoff":
            # set takeoff altitude parameter
            await drone.param.set_param_float("MIS_TAKEOFF_ALT", altitude)
            print("-- Arming")
            await drone.action.arm()

            print("-- Taking off")
            await drone.action.takeoff()

            # await asyncio.sleep(10)

            # print("-- Landing")
            # await drone.action.land()

            # status_text_task.cancel()
        elif action == "land":
            print("-- Landing")
            await drone.action.land()
            # status_text_task.cancel()
        #     land(master)
        elif action == "hold":
            print("-- Holding position")
            # await check_gps_fix_and_arm(drone)
            await fake_hold()
            # status_text_task.cancel()
        else:
            print("Invalid action")
    except Exception as e:
        print(f"ERROR DURING ACTION: {e}")
    finally:
        status_text_task.cancel()
        return
        # stop_flag.set()
        # heartbeat_thread.join()
        # executor.shutdown()
        # master.close()

if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Perform actions with drones.")
    parser.add_argument('--action', type=str, required=True, help='Action to perform: takeoff, land, hold')
    parser.add_argument('--altitude', type=float, default=20, help='Altitude for takeoff')

    args = parser.parse_args()

    # Run the main event loop
    loop = asyncio.get_event_loop()
    loop.run_until_complete(perform_action(args.action, args.altitude))
    print(f"Action: {args.action} completed.")


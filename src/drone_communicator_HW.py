import threading
import os
import time
import csv
import struct
import logging
import select
import subprocess
from concurrent.futures import ThreadPoolExecutor
from src.drone_config import DroneConfig 
from src.params import Params as params
# import serial
import numpy as np
import math
from src.kalman import KalmanFilter_rssi
from pymavlink import mavutil
from enum import Enum

message_types = ['STATUSTEXT', 'UTM_GLOBAL_POSITION', 'ATTITUDE', 'SYS_STATUS']

class Mission(Enum):
    NONE = 0
    DRONE_SHOW_FROM_CSV = 1
    SMART_SWARM = 2
    TAKE_OFF = 10
    LAND = 101
    HOLD = 102
    TEST = 100

processNoise = 0
measurementNoise = 0
class DroneCommunicator_HW:
    def __init__(self, drone_config, param, drones):
        self.drone_config = drone_config
        self.params = param
        self.drones = drones
        self.stop_flag = threading.Event()
        self.nodes = None
        self.executor = ThreadPoolExecutor(max_workers=10)
        self.systemID = drone_config.hw_id + 1
        # self.ser = serial.Serial('/dev/ttyS0', baudrate = 115200)
        self.kf = KalmanFilter_rssi(processNoise, measurementNoise)
        self.ack_count = 0
        print(f"coordinator Sys ID: {self.systemID}")


    def init_mavlink_comms(self):
        # Init Mavlink Connection
        self.master = mavutil.mavlink_connection(f'udp:localhost:{self.params.comms_port}', source_system=self.systemID)
        print(f"Comms: Waiting for Heartbeat at udp:localhost:{self.params.comms_port}")
        self.master.wait_heartbeat()
        print(f'Comms: Heartbeat from system (system {self.master.target_system} component {self.master.target_system})')

    def send_telem(self, packet = None):
        if True:
            test1 = "TEST1"
            
            msg1 = mavutil.mavlink.MAVLink_statustext_message(
                mavutil.mavlink.MAV_SEVERITY_INFO, 
                test1.encode("utf-8")
                # packet[0:40]
            )
            # print(len(packet))
            self.master.mav.send(msg1)
        
    def send_packet_to_node(self, packet):
        self.master.statustext_send(
            mavutil.mavlink.MAV_SECURITY_INFO,
            packet.encode('utf-8')[:50]
        )

    def get_nodes(self):
        if self.nodes is not None:  # modify this line
            return self.nodes
        with open("config.csv", "r") as file:
            self.nodes = list(csv.DictReader(file))  # modify this line
        return self.nodes

    def set_drone_config(self, hw_id, pos_id, state, mission, trigger_time, position, velocity, yaw, battery, last_update_timestamp, rssi):
        if hw_id is not None:
            drone = self.drones.get(hw_id)
        else: 
            drone = self.drone_config
        if pos_id is not None:
            drone.pos_id = pos_id
        if state is not None:
            drone.state = state
        if mission is not None:
            drone.mission = mission
        if trigger_time is not None:
            drone.trigger_time = trigger_time
        if position is not None:
            drone.position = position
        if velocity is not None:
            drone.velocity = velocity
        if yaw is not None:
            drone.yaw = yaw
        if battery is not None:
            drone.battery = battery
        if last_update_timestamp is not None:
            drone.last_update_timestamp = last_update_timestamp
        if rssi is not None:
            drone.rssi = rssi
        self.drones[hw_id] = drone

    def update_state(self, data):
        msg_type = data.get_type()
        if msg_type in message_types: # array of message types are at top of file
            # Ensures Drone_config object will contain position information - Also helps to filter out non-drone systems
            hw_id = data.get_srcSystem()
            logging.debug(f"Received telemetry from Drone {hw_id}")

            # Create a new instance for the drone if not present (including this drone)
            if hw_id not in self.drones and hw_id > 10 and hw_id < 100 and hw_id%10 == 4:
                print(f"New ID: {self.drone_config.hw_id}")
                if hw_id == self.drone_config.hw_id:
                    self.drones[hw_id] = self.drone_config
                else:
                    logging.info(f"Receiving Telemetry from NEW Drone ID= {hw_id}")
                    self.drones[hw_id] = DroneConfig(self.drones, hw_id)

            # Update RSSI Values
            if msg_type == 'STATUSTEXT':
                # print(f"sts: {data}")
                # print("mesg type is status")
                split_string = data.text.split()
                if split_string[0] == 'RSSI':
                    self.set_drone_config(None, None, None, None, None, None, None, None, None, None, split_string[1])
                    print(f"rssi being updated through status message {split_string[1]}")
                elif split_string[0] == 'msn':
                    print(f"{data.text} from {hw_id}")
                    self.decode_status_text(split_string, hw_id)

            # Update Position and Velocity Values
            if msg_type == 'UTM_GLOBAL_POSITION':
                position = {'lat': data.lat / 1E7, 'long': data.lon / 1E7, 'alt': data.alt / 1E7}
                velocity = {'north': data.vx, 'east': data.vy, 'down': data.vz}
                self.set_drone_config(hw_id, None, None, None, None, position, velocity, None, None, data.time, None)
                # print(f"Drone: {hw_id} GPS: {data}\n")
            
            # Update Yaw Values
            if msg_type == 'ATTITUDE':
                self.set_drone_config(hw_id, None, None, None, None, None, None, data.yaw, None, None, None)

            # Update Battery Values
            if msg_type == 'SYS_STATUS':
                self.set_drone_config(hw_id, None, None, None, None, None, None, None, data.voltage_battery, None, None)

    # heading code, rssi aligorthim 
    def get_drone_coords(self, array_hw_id):
       # this function gets all the telemetry and rssi data from the drones
       # it returns a tuple, which has lat, lon, rssi in a int
       # this function returns a wavepoint for the leader to go

        drone_coords = [] # the lat and lon are stored as tuples
        drone_rssi = [] # stored as ints

        for hw_id in array_hw_id:
            # Get drone state based off hw_id
            drone_state = self.drones.get(hw_id)
           
            # Store the lat, lon, and RSSI in a list
            if drone_state != None:
                print(drone_state)
                drone_coords.append((drone_state.position['lat'], drone_state.position['long']))
                drone_rssi.append(drone_state.rssi)
                print(drone_state.rssi) # not accessing correctly
                print(f"coord {drone_coords} rssi : {drone_rssi}")
        # get the vectors
                vectors = self.direction_vectors(drone_coords, drone_rssi)
                estimate_heading = self.estimate_direction(vectors)
                print(f"estimate heading {estimate_heading}")
                # should be leader drone, was not sure how we were going to do this
                # needs to be based of hardware id, or apartently this entire code
                # will only run on leader
                leaderWavePoint = self.calculate_new_waypoint(0, 0, estimate_heading, 40)
                return leaderWavePoint  

    
    # Function to calculate heading from one point to another
    def calculate_heading(self, lat1, lon1, lat2, lon2):
        delta_lon = lon2 - lon1
        delta_lat = lat2 - lat1
        heading = np.degrees(np.arctan2(delta_lon, delta_lat))
        return heading

    def direction_vectors(self, drone_coords, rssi_values):
        vectors = []
        for i in range(len(drone_coords)):
            for j in range(len(drone_coords)):
                if i != j:
                    lat1, lon1 = drone_coords[i]
                    lat2, lon2 = drone_coords[j]
                    rssi_diff = rssi_values[i] - rssi_values[j]
                    weight = np.abs(rssi_diff)
                    heading = self.calculate_heading(lat1, lon1, lat2, lon2)
                    vectors.append((heading, weight))
        return vectors

    def estimate_direction(self, vectors):
        x, y = 0, 0
        for heading, weight in vectors:
            rad = np.radians(heading)
            x += np.cos(rad) * weight
            y += np.sin(rad) * weight
        estimated_heading = np.degrees(np.arctan2(y, x))
        return estimated_heading

    def calculate_new_waypoint(self, lat, lon, heading, distance_meters):
        
        # Convert latitude and heading to radians
        # did some testing in google map, resolution seems doable
        lat_rad = math.radians(lat)
        heading_rad = math.radians(heading)

        # Earth's radius in meters
        R = 6371000

        # Calculate new latitude
        new_lat_rad = lat_rad + (distance_meters / R) * math.cos(heading_rad)
        new_lat = math.degrees(new_lat_rad)

        # Calculate new longitude
        new_lon_rad = math.radians(lon) + (distance_meters / (R * math.cos(lat_rad))) * math.sin(heading_rad)
        new_lon = math.degrees(new_lon_rad)

        return new_lat, new_lon   # hw id of leader = 14
        

    # Fetches the current state of the drone
    # state means telemetry data and rssi values
    # lowkey not a helpful function 
    def get_drone_state(self, hw_id):
        drone = self.drones.get(hw_id)
        if drone is not None:
            drone_state = {
            "hw_id": int(drone.hw_id),
            "pos_id": int(drone.config['pos_id']),
            "state": int(drone.state),
            "mission": int(drone.mission),
            "trigger_time": int(drone.trigger_time),
            "position_lat": drone.position['lat'],
            "position_long": drone.position['long'],
            "position_alt": drone.position['alt'],
            "velocity_north": drone.velocity['north'],
            "velocity_east": drone.velocity['east'],
            "velocity_down": drone.velocity['down'],
            "yaw": drone.yaw,
            "battery_voltage": drone.battery,
            "follow_mode": int(drone.swarm['follow']),
            "update_time": int(drone.last_update_timestamp),
            "RSSI": drone.rssi
            }
            return drone_state


    def send_drone_state(self):
        while not self.stop_flag.is_set():
            # self.get_drone_coords([24])
            # Get RSSI Data From LoRa and Send via STATUSTEXT
            # if self.ser.is_open:
            #     # decode the message        
            #     data_message = self.ser.readline().decode('utf-8').strip() # this is the message
            #     if data_message:
            #         data_message_filter = data_message.split(',')
            #         rssiVal = int(data_message_filter[-2])
             
            #         self.drone_config.rssi = self.kf.filter(rssiVal) # put kf filter 
            #         # config = self.drones.get(self.drone_config.hw_id)
            #         # config.rssi  = self.kf.filter(rssiVal)
            #         print(self.drone_config.rssi)
                    
            #         #print(f"rssi value {self.drone_config.rssi}")
            #         rssi_tosend = f"RSSI { self.drone_config.rssi}"
            #         self.master.mav.statustext_send(
            #             mavutil.mavlink.MAV_SEVERITY_INFO,
            #             rssi_tosend.encode('utf-8')
            #         )
            #         # KF_rssi = self.kf.filter(rssiVal)
            #         # KF_var = self.kf.get_cov()

            self.send_drone_ack() # send drone ack periodically as needed

            time.sleep(self.params.TELEM_SEND_INTERVAL)
        
    def read_packets(self):
        while not self.stop_flag.is_set():
            ready = select.select([self.master.fd], [], [], self.params.income_packet_check_interval)
            if ready[0]:
                msg = self.master.recv_match()
                if (msg):
                    self.update_state(msg)
                    # print(msg)

            if self.drone_config.mission == 2 and self.drone_config.state != 0 and int(self.drone_config.swarm.get('follow')) != 0:
                    self.drone_config.calculate_setpoints()

    def decode_status_text(self, text, sys_id): # input text is already split
        sys_id_list = [obj.hw_id for obj in self.drones.values()]
        components = text # format: msn_#_[ack] [ack] is only added if sent from a drone. ignore if from gcs
        mission_code = [int(component) for component in components if component.isdigit()][0]
        if sys_id == 4: # this will arrive multiple times. Change to idle mode once and ignore anything after that
            print("recv new mission")
            if self.drone_config.gcs_msn != mission_code:
                print(f"mission code: {mission_code} not same curr: {self.drone_config.mission}")
                if mission_code == Mission.SMART_SWARM.value or mission_code == Mission.DRONE_SHOW_FROM_CSV.value:
                    if self.drone_config.mission != Mission.HOLD.value:
                        self.set_drone_config(None, None, None, Mission.HOLD.value, None, None, None, None, None, None, None)
                elif self.drone_config.prev_mission != mission_code:
                    self.set_drone_config(None, None, None, mission_code, None, None, None, None, None, None, None)
                for drone_object in self.drones.values():
                    drone_object.gcs_msn = mission_code
                    drone_object.gcs_msn_ack = False
                self.drone_config.gcs_msn_ack = True
                self.ack_count = 0
                    
        elif sys_id - 1 in sys_id_list:
            if mission_code and components[2] == 'ack':
                self.drones[sys_id - 1].gcs_msn_ack = True
                print(f"got ack from drone {sys_id}")


    def start_communication(self):
        self.telemetry_thread = threading.Thread(target=self.send_drone_state)
        self.command_thread = threading.Thread(target=self.read_packets)
        self.telemetry_thread.start()
        self.command_thread.start()

    def stop_communication(self):
        self.stop_flag.set()
        self.telemetry_thread.join()
        self.command_thread.join()
        self.executor.shutdown()

    def check_all_drone_ack(self): # simple loops that checks all drone acks
        for drone in self.drones.values():
            if drone.gcs_msn_ack is False:
                self.ack_count = 0 # reset the ack count until all drones acks have arrived
                return False
        if self.drone_config.mission != self.drone_config.gcs_msn and self.drone_config.prev_mission != self.drone_config.gcs_msn:
            print(f"Changing to mission from gcs: {self.drone_config.gcs_msn}")
            self.drone_config.mission = self.drone_config.gcs_msn
        return True

    def send_drone_ack(self): # broadcast 10 times
        self.check_all_drone_ack()
        if self.ack_count < 10 and self.drone_config.gcs_msn_ack is True and self.drone_config.gcs_msn != 0:
            # print(f"sending drone ack {self.ack_count}")
            self.master.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,
                f"msn {self.drone_config.gcs_msn} ack".encode('utf-8')
            )
            self.ack_count += 1
		# all drones will keep on sending their acks until


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
import serial

from pymavlink import mavutil

class DroneCommunicator_HW:
    def __init__(self, drone_config, param, drones):
        self.drone_config = drone_config
        self.params = param
        self.drones = drones
        self.stop_flag = threading.Event()
        self.nodes = None
        self.executor = ThreadPoolExecutor(max_workers=10)
        self.systemID = drone_config.hw_id * 10 + 5
        self.ser = serial.Serial('/dev/ttyS0', baudrate = 115200)
        # self.kf = KalmanFilter(processNoise, measurementNoise)
        # print(f"coordinator Sys ID: {self.systemID}")


    def init_mavlink_comms(self):
        # Init Mavlink Connection
        self.master = mavutil.mavlink_connection(f'udp:localhost:{self.params.comms_port}', source_system=self.systemID)
        print(f"Comms: Waiting for Heartbeat at udp:localhost:{self.params.comms_port}")
        self.master.wait_heartbeat()
        print(f'Comms: Heartbeat from system (system {self.master.target_system} component {self.master.target_system})')

    def send_telem(self, packet):
        if True:
            test1 = "TEST1"
            test2 = "TEST2"
            
            msg1 = mavutil.mavlink.MAVLink_statustext_message(
                mavutil.mavlink.MAV_SEVERITY_INFO, 
                test1.encode("utf-8")
                # packet[0:40]
            )
            msg2 = mavutil.mavlink.MAVLink_statustext_message(
                mavutil.mavlink.MAV_SEVERITY_INFO, 
                test2.encode("utf-8")
                # packet[40:81]
            )
            # print(len(packet))
            self.master.mav.send(msg1)
            self.master.mav.send(msg2)
        
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
        drone = self.drones.get(hw_id)
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
        if msg_type == "STATUSTEXT" or msg_type == "UTM_GLOBAL_POSITION" or msg_type == "ATTITUDE" or msg_type == "SYS_STATUS":
            # Ensures Drone_config object will contain position information - Also helps to filter out non-drone systems
            hw_id = data.get_srcSystem()
            logging.debug(f"Received telemetry from Drone {hw_id}")

            # Create a new instance for the drone if not present
            if hw_id not in self.drones:
                logging.info(f"Receiving Telemetry from NEW Drone ID= {hw_id}")
                self.drones[hw_id] = DroneConfig(self.drones, hw_id)

            # Update RSSI Values
            if msg_type == 'STATUSTEXT':
                split_string = data.text.split()
                if split_string[0] == 'RSSI':
                    self.set_drone_config(None, None, None, None, None, None, None, None, None, None, split_string[1])

            # Update Position and Velocity Values
            if msg_type == 'UTM_GLOBAL_POSITION':
                position = {'lat': data.lat / 1E7, 'long': data.lon / 1E7, 'alt': data.alt / 1E7}
                velocity = {'north': data.vx, 'east': data.vy, 'down': data.vz}
                self.set_drone_config(hw_id, None, None, None, None, position, velocity, None, None, data.time, None)
            
            # Update Yaw Values
            if msg_type == 'ATTITUDE':
                self.set_drone_config(hw_id, None, None, None, None, None, None, data.yaw, None, None, None)

            # Update Battery Values
            if msg_type == 'SYS_STATUS':
                self.set_drone_config(hw_id, None, None, None, None, None, None, None, data.voltage_battery, None, None)

    def process_packet(self, data):
        header, terminator = struct.unpack('BB', data[0:1] + data[-1:])
        
        if header == 55 and terminator == 66 and len(data) == self.params.command_packet_size:
            header, hw_id, pos_id, mission, state, trigger_time, terminator = struct.unpack(self.params.command_struct_fmt, data)
            logging.info(f"Received command from GCS: hw_id: {hw_id}, pos_id: {pos_id}, mission: {mission}, state: {state}, trigger_time: {trigger_time}")

            self.drone_config.hw_id = hw_id
            self.drone_config.pos_id = pos_id
            self.drone_config.state = state
            self.drone_config.trigger_time = trigger_time

            # Handle TAKE_OFF with altitude
            if 10 <= mission < 60:
                altitude = mission - 10
                if altitude > 50:
                    altitude = 50
                print(f"Takeoff command received. Altitude: {altitude}m")
                
                # Update mission code to default TAKE_OFF code after extracting altitude
                self.drone_config.mission = mission  # Change this to your default TAKE_OFF code
                
            elif mission == 1:
                print("Drone Show command received.")
                self.drone_config.mission = mission
                
            elif mission == 2:
                print("Smart Swarm command received.")
                self.drone_config.mission = mission
                
            elif mission == self.params.Mission.LAND.value:
                print("Land command received.")
                self.drone_config.mission = mission
                
            elif mission == self.params.Mission.HOLD.value:
                print("Hold command received.")
                self.drone_config.mission = mission
                
            elif mission == self.params.Mission.TEST.value:
                print("Test command received.")
                self.drone_config.mission = mission
            else:
                print(f"Unknown mission command received: {mission}")
                self.drone_config.mission = self.params.Mission.NONE.value




            # Add additional logic here to handle the received command
        elif header == 77 and terminator == 88 and len(data) == self.params.telem_packet_size:
            # Decode the data
            header, hw_id, pos_id, state, mission, trigger_time, position_lat, position_long, position_alt, velocity_north, velocity_east, velocity_down, yaw, battery_voltage, follow_mode, update_time ,  terminator = struct.unpack(self.params.telem_struct_fmt, data)
            logging.debug(f"Received telemetry from Drone {hw_id}")

            if hw_id not in self.drones:
                # Create a new instance for the drone
                logging.info(f"Receiving Telemetry from NEW Drone ID= {hw_id}")
                self.drones[hw_id] = DroneConfig(self.drones, hw_id)

            position = {'lat': position_lat, 'long': position_long, 'alt': position_alt}
            velocity = {'north': velocity_north, 'east': velocity_east, 'down': velocity_down}
            self.set_drone_config(hw_id, pos_id, state, mission, trigger_time, position, velocity, yaw, battery_voltage, update_time)
            # Add processing of the received telemetry data here
        else:
            logging.error(f"Received packet of incorrect size or header. Got {len(data)} bytes.")
    
    def receive(self):
        msg = self.master.recv_match()
        if msg and msg.get_type():
            print(msg)
        else:
            return

    def receive(self, _type):
        msg = self.master.recv_match(type=_type)
        if msg and msg.get_type() == type:
            print(msg)
        else:
            return
        
    # Fetches the current state of the drone
    def get_drone_state(self, hw_id):
        drone = self.drones.get(hw_id)
        if drone is not None:
            drone_state = {
            "hw_id": int(drone.hw_id),
            "pos_id": int(drone.pos_id),
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
            # "follow_mode": int(drone.swarm['follow']),
            # "update_time": int(drone.last_update_timestamp),
            "RSSI": drone.rssi
            }
            print(drone_state)

            return drone_state


    def send_drone_state(self):
        print("HERE")
        while not self.stop_flag.is_set():
            # self.get_drone_state(2)
            # Get RSSI Data From LoRa and Send via STATUSTEXT
            if self.ser.is_open:
                    # decode the message        
                    data_message = self.ser.readline().decode('utf-8').strip() # this is the message
                    if data_message:
                        data_message_filter = data_message.split(',')
                        rssiVal = data_message_filter[-2]
                        self.drone_config.rssi = rssiVal
                        
                        print(f"rssi value {self.drone_config.rssi}")
                        rssi_tosend = f"RSSI { self.drone_config.rssi}"
                        self.master.mav.statustext_send(
                            mavutil.mavlink.MAV_SEVERITY_INFO,
                            rssi_tosend.encode('utf-8')
                        )
                        # KF_rssi = self.kf.filter(rssiVal)
                        # KF_var = self.kf.get_cov()

            time.sleep(self.params.TELEM_SEND_INTERVAL)
        
    def read_packets(self):
        while not self.stop_flag.is_set():
            ready = select.select([self.master.fd], [], [], self.params.income_packet_check_interval)
            if ready[0]:
                msg = self.master.recv_match()
                
                self.update_state(msg)


            if self.drone_config.mission == 2 and self.drone_config.state != 0 and int(self.drone_config.swarm.get('follow')) != 0:
                    self.drone_config.calculate_setpoints()

    def decode_status_text(self, text, sys_id):
        components = text.split(" ")
        if sys_id == 14:
            return None
        if sys_id == 24:
            return None
        if sys_id == 34:
            return None
        codes = [int(components) for component in components if component.isdigit()]
        print(sys_id)
        print(codes)

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

    def drone_ack(self, cmd, msn):
        # add code here or modification to send an ack. Also, before this runs, coordinator should enter idle mode or
        test1 = "cmd " + str(cmd) + " ack 1 msn " + str(msn)
        count = 0
        
        msg1 = mavutil.mavlink.MAVLink_statustext_message(
            mavutil.mavlink.MAV_SEVERITY_INFO, 
            test1.encode("utf-8")
            # packet[0:40]
        )
        # print(len(packet))
        for i in range(10):
            self.master.mav.send(msg1)
            time.sleep(1)


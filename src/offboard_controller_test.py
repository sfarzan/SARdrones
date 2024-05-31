import logging
import asyncio
import subprocess
import psutil  # Import psutil to find and kill existing MAVSDK server processes
from mavsdk import System
from mavsdk.offboard import AccelerationNed, OffboardError, PositionNedYaw, VelocityNedYaw
import functions.global_to_local
import csv
import time


class OffboardController:
    """
    This class encapsulates the logic to control a drone in offboard mode
    using the MAVSDK library.
    """
    def __init__(self, drone_config, params, mavsdk_server_address='localhost'):
        self.drone_config = drone_config
        self.offboard_follow_update_interval = 0.2
        self.port = 50050 + int(self.drone_config.hw_id)
        self.upd_port = params.mavsdk_port
        self.mavsdk_server_address = mavsdk_server_address
        self.is_offboard = False
        self.mavsdk_server_process = None
        self.use_filter = True
        self.use_acceleration = True
        self.params = params
        self.global_position_telemetry = None

        
        self.stop_existing_mavsdk_server(self.port)

        

    def start_swarm(self):
        self.is_offboard = True

    def calculate_follow_setpoint(self):
        if self.drone_config.mission == 2 and self.drone_config.state != 0 and int(self.drone_config.swarm.get('follow')) != 0:
            self.drone_config.calculate_setpoints()

    def stop_existing_mavsdk_server(self, port):
        for proc in psutil.process_iter():
            try:
                if "mavsdk_server" in proc.name():
                    for conns in proc.connections(kind='inet'):
                        if conns.laddr.port == port:
                            proc.terminate()
                            logging.info(f"Terminated existing MAVSDK server on port {port}")
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass


    async def connect(self):
        self.drone = System(sysid=200+self.params.hw_id)
        await self.drone.connect(system_address=f"udp://:{self.params.mavsdk_port}")
        drone_id_param = await self.drone.param.get_param_int("MAV_SYS_ID")

        while drone_id_param != self.params.hw_id:
            print(f"wrong id: {drone_id_param} vs {self.params.hw_id}")
            await self.drone.connect(system_address=f"udp://:{self.params.mavsdk_port}")
            # get the system id parameter
            drone_id_param = await self.drone.param.get_param_int("MAV_SYS_ID")
        print(f"sysid = {self.drone._sysid} drone_id_param = {drone_id_param}")
        
        # self.drone = System(sysid=200 + self.params.hw_id)
        # await self.drone.connect(f'udp://:{self.upd_port}')

        logging.info("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                logging.info("Drone discovered")
                break
        asyncio.ensure_future(self.get_global_position_telemetry())

    async def set_initial_position(self):
        initial_pos = PositionNedYaw(
            self.drone_config.position_setpoint_NED['north'], 
            self.drone_config.position_setpoint_NED['east'], 
            self.drone_config.position_setpoint_NED['down'], 
            self.drone_config.yaw_setpoint)
        await self.drone.offboard.set_position_ned(initial_pos)
        logging.info(f"Initial setpoint: {initial_pos}")

    async def start_offboard(self):
        """
        Start the offboard mode.
        """
        try:
            await self.drone.offboard.start()
            logging.info("Offboard started.")
            self.is_offboard = True
        except OffboardError as error:
            logging.error(f"Starting offboard mode failed with error code: {error._result.result}")
            return

    async def maintain_setpoints(self):
        """Maintain position, velocity, and optionally acceleration."""
        try:
            while True:
                if self.use_filter==True:
                    state = self.drone_config.kalman_filter.get_current_state()
                    pos = state['position']
                    vel = state['velocity']
                    acc = [state['acceleration']['north'], state['acceleration']['east'], state['acceleration']['down']]
                    logging.debug(f"States: | Position: {pos} | Velocity: {vel} | Acceleration: {acc}")
                else:
                    # Use raw setpoints
                    pos = self.drone_config.position_setpoint_NED
                    vel = self.drone_config.velocity_setpoint_NED
                    acc = [0, 0, 0]  # Assume zero acceleration

                pos_ned_yaw = PositionNedYaw(pos['north'], pos['east'], pos['down'], self.drone_config.yaw_setpoint)
                vel_ned_yaw = VelocityNedYaw(vel['north'], vel['east'], vel['down'], self.drone_config.yaw_setpoint)  # Use 'north', 'east', 'down' keys

                if self.use_acceleration == True:
                    acc_ned = AccelerationNed(acc[0], acc[1], acc[2])
                    await self.drone.offboard.set_position_velocity_acceleration_ned(pos_ned_yaw, vel_ned_yaw, acc_ned)
                else:
                    await self.drone.offboard.set_position_velocity_ned(pos_ned_yaw, vel_ned_yaw)

                logging.debug(f"Maintaining setpoints | Position: {pos} | Velocity: {vel} | Acceleration: {acc}")

                if self.drone_config.mission in [1, 101]:
                    break

                await asyncio.sleep(0.2)  # Update rate of 200 ms

        except OffboardError as e:
            logging.error(f"Offboard Error: {e}", exc_info=True)
        except Exception as e:
            logging.error(f"An unexpected error occurred: {e}", exc_info=True)
        finally:
            # Stop the MAVSDK server and set offboard flag to False
            await self.stop_offboard()
            self.drone._stop_mavsdk_server()


    async def stop_offboard(self):
        """
        Stop the offboard mode.
        """
        await self.drone.offboard.stop()
        logging.info("Offboard stopped.")
        self.is_offboard = False

    async def land_drone(self):
        """
        Land the drone.
        """
        if self.is_offboard:
            await self.stop_offboard()
            await asyncio.sleep(1)
        await self.drone.action.land()
        logging.info("Drone landing.")

    async def start_offboard_follow(self):
        """
        Initialize and execute offboard following operations.
        """
        await self.connect()
        await self.set_initial_position()
        await self.start_offboard()
        await self.maintain_setpoints()

    async def init_leader_checks(self):
        """
        Initialize Leader and Check for SAR
        """
        SAR = False

        await self.connect()
        if SAR is True:
           print("SAR Initialized")
           await self.start_offboard()
           # Add Function for Offboard Control for SAR Leader
        if SAR is False:
            print("Manual Control Initialized")
            await self.stop_offboard()


    # ----- CSV SEARCH MODE FUNCTIONS ----- #
    async def start_csv(self):
        """
        Initialize and execute offboard following operations.
        """
        await self.connect()
        await self.set_initial_position()
        await self.start_offboard()
        await self.run_drone_csv()


    async def run_drone_csv(self):
        if self.params.SEPERATE_CSV:
            filename = "../FlightPaths/swarm/processed/Drone " + str(self.params.hw_id) + ".csv"    
        else:
            filename = "shapes/active.csv"

        try:
            altitude_offset = 0
            trajectory_offset = (0, 0, 0)
            waypoints = self.read_trajectory_file(filename, trajectory_offset, altitude_offset)
            home_position_NED = (self.drone_config.position_setpoint_NED['north'], self.drone_config.position_setpoint_NED['east'], self.params.DEFAULT_Z)
            home_position = self.global_position_telemetry
            print("HERE")
            await self.perform_trajectory(self.params.hw_id, self.drone, waypoints, home_position, home_position_NED)
        except Exception as e:
            print(f"ERROR INITALIZING CSV: {e}")


        # Stop offboard mode
        await self.stop_offboard()

    def read_trajectory_file(self, filename, trajectory_offset, altitude_offset):
        waypoints = []
        with open(filename, newline="") as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                t = float(row["t"])
                px = float(row["px"]) + trajectory_offset[0]
                py = float(row["py"]) + trajectory_offset[1]
                pz = float(row["pz"]) + trajectory_offset[2] - altitude_offset
                vx = float(row["vx"])
                vy = float(row["vy"])
                vz = float(row["vz"])
                ax = float(row["ax"])
                ay = float(row["ay"])
                az = float(row["az"])
                yaw = float(row["yaw"])
                mode_code = int(row["mode"])  # Assuming the mode code is in a column named "mode"
                waypoints.append((t, px, py, pz, vx, vy, vz, ax, ay, az,yaw, mode_code))
        return waypoints

    async def perform_trajectory(self, drone_id, drone, waypoints, home_position,home_position_NED):
        print(f"-- Performing trajectory {drone_id}")
        total_duration = waypoints[-1][0]
        t = 0
        last_mode = 0
        last_waypoint_index = 0

        try: 
            while t <= total_duration:
                if self.drone_config.mission > 3:
                    break
                
                pos = self.drone_config.position_setpoint_NED
                vel = self.drone_config.velocity_setpoint_NED
                acc = [0, 0, 0]  # Assume zero acceleration

                
                
                actual_position = self.global_position_telemetry

                local_ned_position = functions.global_to_local.global_to_local(actual_position, home_position)
                
                current_waypoint = None
                for i in range(last_waypoint_index, len(waypoints)):
                    if t <= waypoints[i][0]:
                        current_waypoint = waypoints[i]
                        last_waypoint_index = i
                        break

                if (self.params.SEPERATE_CSV == True):
                    position = tuple(a-b for a, b in zip(current_waypoint[1:4], home_position_NED))
                else:
                    position = current_waypoint[1:4]

                
                velocity = current_waypoint[4:7]
                acceleration = current_waypoint[7:10]
                yaw = current_waypoint[10]
                mode_code = current_waypoint[-1]
                if last_mode != mode_code:
                    print(f"Drone id: {drone_id+1}: Mode number: {mode_code}")
                    last_mode = mode_code
                        
                await drone.offboard.set_position_velocity_acceleration_ned(
                    PositionNedYaw(*position, yaw),
                    VelocityNedYaw(*velocity, yaw),
                    AccelerationNed(*acceleration)
                )

                await asyncio.sleep(self.params.STEP_TIME)
                t += self.params.STEP_TIME
                
                if int(t/self.params.STEP_TIME) % 100 == 0:
                    deviation = [(a - b) for a, b in zip(position, [local_ned_position.north_m, local_ned_position.east_m, local_ned_position.down_m])]
                    if self.params.SHOW_DEVIATIONS == True:
                        print(f"Drone {drone_id+1} Deviations: {round(deviation[0], 1)} {round(deviation[1], 1)} {round(deviation[2], 1)}")
        except Exception as e:
            print(f"ERROR IN MAIN LOOP{e}")

        print(f"-- Shape completed {drone_id+1}")

    async def get_global_position_telemetry(self):
        async for global_position in self.drone.telemetry.position():
            self.global_position_telemetry = global_position
            pass

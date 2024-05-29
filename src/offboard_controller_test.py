import logging
import asyncio
import subprocess
import psutil  # Import psutil to find and kill existing MAVSDK server processes
from mavsdk import System
from mavsdk.offboard import AccelerationNed, OffboardError, PositionNedYaw, VelocityNedYaw
from src.params import Params as params
from pymavlink import mavutil

class OffboardController:
    """
    This class encapsulates the logic to control a drone in offboard mode
    using the MAVSDK library.
    """
    def __init__(self, drone_config, mavsdk_server_address='localhost'):
        self.drone_config = drone_config
        self.offboard_follow_update_interval = 0.2
        self.port = params.mavsdk_port
        self.upd_port = 14550 + int(self.drone_config.hw_id)
        self.mavsdk_server_address = mavsdk_server_address
        self.is_offboard = False
        self.mavsdk_server_process = None
        self.use_filter = True
        self.use_acceleration = True
        
        self.stop_existing_mavsdk_server(self.port)

    def start_swarm(self):
        self.is_offboard = True

    def calculate_follow_setpoint(self):
        if self.drone_config.mission == 2 and self.drone_config.state != 0 and int(self.drone_config.swarm.get('follow')) != 0:
            self.drone_config.calculate_setpoints()

    def stop_existing_mavsdk_server(self, port):
        try:
            self.master.close()
            logging.info(f"Terminated existing pymavlink on port {port}")
        except Exception as e:
            logging.error(f"nothing closed: {e}")
            return None

    def start_mavsdk_server(self, port):
        try:
            # Init Mavlink Connection
            self.master = mavutil.mavlink_connection(f'udp:localhost:{self.params.mavsdk_port}', source_system=self.systemID)
            print(f"Comms: Waiting for Heartbeat at udp:localhost:{self.params.comms_port}")
            self.master.wait_heartbeat()
            print(f'Comms: Heartbeat from system (system {self.master.target_system} component {self.master.target_system})')

        except Exception as e:
            logging.error(f"Error starting pymavlink: {e}")
            return None

    def stop_mavsdk_server(self):
        try:
            self.master.close()
            logging.info("pymavlink terminated.")
        except Exception as e:
            logging.error(f"Error stopping pymavlink: {e}")


    async def connect(self):
        self.start_mavsdk_server(self.port)
        # self.drone = System(self.mavsdk_server_address, self.port)
        
        # await self.drone.connect(f'udp://:{self.upd_port}')

        logging.info("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                logging.info("Drone discovered")
                break
        # Removed redundant call to start_mavsdk_server

    async def start_offboard(self):
        """
        Start the offboard mode using pymavlink.
        """
        try:
            master = self.master  # Ensure master is the pymavlink connection object

            # Set the mode to OFFBOARD
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                0, 6, 0, 0, 0, 0)  # The '6' here corresponds to OFFBOARD mode for PX4

            # Confirm the mode is set
            ack = master.recv_match(type='COMMAND_ACK', blocking=True)
            if ack.command != mavutil.mavlink.MAV_CMD_DO_SET_MODE or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                logging.error(f"Setting offboard mode failed with result: {ack.result}")
                return

            logging.info("Offboard mode set.")

            # Optionally arm the drone if it's not armed
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,  # 1 to arm, 0 to disarm
                0, 0, 0, 0, 0, 0)

            # Confirm the arming
            ack = master.recv_match(type='COMMAND_ACK', blocking=True)
            if ack.command != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                logging.error(f"Arming the drone failed with result: {ack.result}")
                return

            logging.info("Drone armed.")

            # Set the offboard flag
            self.is_offboard = True

        except Exception as e:
            logging.error(f"Starting offboard mode failed with error: {e}", exc_info=True)
            return


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
                    await self.send_position_velocity_acceleration_ned(pos_ned_yaw, vel_ned_yaw, acc_ned)
                else:
                    await self.send_position_velocity_ned(pos_ned_yaw, vel_ned_yaw)

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
            self.stop_mavsdk_server()



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
    
    async def send_position_velocity_acceleration_ned(self, pos_ned_yaw, vel_ned_yaw, acc_ned):
        self.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            self.master.target_system,  # target_system
            self.master.target_component,  # target_component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # coordinate frame
            0b0000111111000111,  # type_mask (ignore position)
            pos_ned_yaw.north, pos_ned_yaw.east, pos_ned_yaw.down,  # x, y, z positions
            vel_ned_yaw.north, vel_ned_yaw.east, vel_ned_yaw.down,  # x, y, z velocities
            acc_ned.north, acc_ned.east, acc_ned.down,  # x, y, z accelerations
            0, 0, pos_ned_yaw.yaw  # yaw, yaw_rate (0 for yaw_rate)
        )

    async def send_position_velocity_ned(self, pos_ned_yaw, vel_ned_yaw):
        self.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            self.master.target_system,  # target_system
            self.master.target_component,  # target_component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # coordinate frame
            0b0000111111000111,  # type_mask (ignore acceleration)
            pos_ned_yaw.north, pos_ned_yaw.east, pos_ned_yaw.down,  # x, y, z positions
            vel_ned_yaw.north, vel_ned_yaw.east, vel_ned_yaw.down,  # x, y, z velocities
            0, 0, 0,  # x, y, z accelerations (ignored)
            0, 0, pos_ned_yaw.yaw  # yaw, yaw_rate (0 for yaw_rate)
        )

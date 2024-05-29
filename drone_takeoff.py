import time
from pymavlink import mavutil
from src import params as params

# Establish a connection to the drone
# Replace 'COM3' and '57600' with your serial port and baud rate
# Or use 'udpin:0.0.0.0:14550' for a UDP connection
connection = mavutil.mavlink_connection(f'udp://:{params.mavsdk_port}')

# Wait for the heartbeat message to find the system ID
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

# Arm the drone
def arm_drone():
    connection.mav.command_long_send(
        connection.target_system, 
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

    # Wait until the drone is armed
    while True:
        connection.motors_armed_wait()
        print("Motors armed!")
        break

# Take off
def takeoff(altitude):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, altitude)
    
    print("Takeoff command sent!")

# Main sequence
def main():
    arm_drone()
    time.sleep(1)
    takeoff(10)  # Take off to 10 meters altitude

if __name__ == "__main__":
    main()

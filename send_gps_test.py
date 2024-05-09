#!/usr/bin/env python3

import asyncio
import time
from mavsdk import System
from mavsdk.server_utility import StatusTextType

"""
This example shows how to use server_utility plugin to send status messages.

The messages will appear in GCS log. In order to debug messages you can use:
  - QGroundControll MAVLink Inspector
  - Wireshark (https://mavlink.io/en/guide/wireshark.html)

In this example we are changing sysid in order to show our message along the
other messages in QGroundControll interface.
"""


async def run():

    drone = System(sysid=24)
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            for i in range(10):
                print(f"-- Connected to drone! i: " + str(i))
                async for position in drone.telemetry.position():
                    await drone.server_utility.send_(
                        StatusTextType.INFO, f"GPS INFO: {position}")
                    time.sleep(1)
            #break

if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())

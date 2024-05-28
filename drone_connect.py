import asyncio
from mavsdk import System
from mavsdk.server_utility import StatusTextType

async def run():

    drone = System(sysid=1)
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            await drone.server_utility.send_status_text(
                StatusTextType.INFO, "Hello world!")
            break

if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())

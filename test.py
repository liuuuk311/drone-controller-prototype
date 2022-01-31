import argparse
import asyncio
import logging
import os
from mavsdk import System

logFormatter = " %(asctime)s %(name)s - %(levelname)s: %(message)s"
logging.basicConfig(level=logging.DEBUG, format=logFormatter)

parser = argparse.ArgumentParser(description="Control the drone")
parser.add_argument(
    "--takeoff",
    dest="takeoff",
    action="store_true",
    help="Run the test procedure for uploading the mission",
)
args = parser.parse_args()


async def test_takeoff():
    # Init the drone
    drone = System()
    address = os.getenv("ARDU_SERIAL_CONN", "udp://:14540")
    await drone.connect(system_address=address)

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break

    # Start parallel tasks
    print_altitude_task = asyncio.ensure_future(print_altitude(drone))
    print_flight_mode_task = asyncio.ensure_future(print_flight_mode(drone))

    running_tasks = [print_altitude_task, print_flight_mode_task]
    termination_task = asyncio.ensure_future(observe_is_in_air(drone, running_tasks))

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("Global position estimate ok")
            break

    # Execute the maneuvers
    print("-- Arming")
    await drone.action.arm()
    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(5)

    print("-- Landing")
    await drone.action.land()

    # Wait until the drone is landed (instead of exiting after 'land' is sent)
    await termination_task


async def print_altitude(drone):
    """ Prints the altitude when it changes """

    previous_altitude = None

    async for position in drone.telemetry.position():
        altitude = round(position.relative_altitude_m)
        if altitude != previous_altitude:
            previous_altitude = altitude
            print(f"Altitude: {altitude}")


async def print_flight_mode(drone):
    """ Prints the flight mode when it changes """

    previous_flight_mode = None

    async for flight_mode in drone.telemetry.flight_mode():
        if flight_mode != previous_flight_mode:
            previous_flight_mode = flight_mode
            print(f"Flight mode: {flight_mode}")


async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()
            return


async def run():
    if args.takeoff:
        await test_takeoff()


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())

import asyncio
import logging
import os
import argparse

from core.controller import DroneController

logFormatter = ' %(asctime)s %(name)s - %(levelname)s: %(message)s'
logging.basicConfig(level=logging.DEBUG, format=logFormatter)

parser = argparse.ArgumentParser(description='Control the drone')
parser.add_argument('--test', dest='test', help='Run the test procedure')

args = parser.parse_args()


async def run():
    address = os.getenv("ARDU_SERIAL_CONN", "udp://:14540")
    controller = DroneController(connection_address=address)
    await controller.run()

async def test():
    address = os.getenv("ARDU_SERIAL_CONN", "udp://:14540")
    controller = DroneController(connection_address=address)
    await controller.test()


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    if args.test: 
        loop.run_until_complete(test())
    else:
        loop.run_until_complete(run())

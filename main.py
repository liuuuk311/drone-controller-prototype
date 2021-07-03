import asyncio
import logging
import os

from core.controller import DroneController

logFormatter = ' %(asctime)s %(name)s - %(levelname)s: %(message)s'
logging.basicConfig(level=logging.DEBUG, format=logFormatter)


async def run():
    address = os.getenv("ARDU_SERIAL_CONN", "udp://:14540")
    controller = DroneController(connection_address=address)
    await controller.run()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())

import logging

class Updater:
    def __init__(self):
        self.ground_station_address = "127.0.0.1"
        self.ground_station_port = 2311
        self.logger = logging.getLogger(__name__)

    @staticmethod
    async def download_next_mission_plan() -> str:
        return "data/missions/mission.plan"

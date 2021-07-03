import asyncio
import logging
from enum import Enum

from mavsdk import System
from mavsdk.action import ActionError

from aiofsm import StateMachine, transition

from core.exceptions import ConnectionFailedTooManyTimes
from core.updater import Updater


class DroneState(Enum):
    BOOTSTRAPPING = "bootstrapping"
    READY = "ready"
    FLYING = "flying"
    CHARGING = "charging"


class DroneStateMachine(StateMachine):

    def __init__(self):
        self.state = DroneState.BOOTSTRAPPING
        super().__init__()

    @property
    def is_bootstrapping(self):
        return self.state == DroneState.BOOTSTRAPPING

    @property
    def is_ready(self):
        return self.state == DroneState.READY

    @property
    def is_flying(self):
        return self.state == DroneState.FLYING

    @property
    def is_charging(self):
        return self.state == DroneState.CHARGING


class AsyncTaskManagerMixin:
    def __init__(self):
        try:
            self.running_tasks
        except AttributeError:
            raise ValueError("Need to set a running_tasks: list instance variable")

    def add_new_bg_task(self, task):
        new_task = asyncio.ensure_future(task)
        self.running_tasks.append(new_task)

    async def terminate_running_tasks(self):
        for task in self.running_tasks:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        await asyncio.get_event_loop().shutdown_asyncgens()
        self.running_tasks = []


class DroneController(DroneStateMachine, AsyncTaskManagerMixin):
    def __init__(self, connection_address: str, max_retries: int = 3):
        super().__init__()
        self._connection_address = connection_address
        self.drone = System()
        self.updater = Updater()
        self.logger = logging.getLogger(__name__)
        self.is_simulation = False if "serial://" in connection_address else True
        self.flight_level = 5
        self.max_connection_retries = max_retries
        self.connection_attempts = 0
        self.running_tasks = []

    @transition(
        source=DroneState.BOOTSTRAPPING,
        target=DroneState.READY,
        on_error=DroneState.BOOTSTRAPPING,
        exception=ConnectionFailedTooManyTimes
    )
    async def bootstrap(self):
        """ The boostrap phase is needed to connect the software to the drone via MavLink telemetry.
        This should be the first method to be called """
        self.logger.info("Bootstrapping...")
        await self.drone.connect(self._connection_address)

        async for state in self.drone.core.connection_state():
            self.logger.debug("Waiting for connection")

            if state.is_connected is True:
                self.logger.info("The system is now connected")
                break

            self.logger.debug("Failed to connect. Retrying...")
            self.connection_attempts += 1

            if self.connection_attempts > self.max_connection_retries:
                self.logger.debug("Tried too many times to connect")
                raise ConnectionFailedTooManyTimes(attempts=self.connection_attempts)

        self.logger.info("Setting defaults")
        await self.drone.action.set_takeoff_altitude(self.flight_level)
        await self.drone.action.set_return_to_launch_altitude(self.flight_level)

        await self.update()
        self.logger.info("Bootstrapping completed")

    @transition(
        source=DroneState.CHARGING,
        target=DroneState.READY,
    )
    async def observe_charging(self):
        """ Monitor the battery percentage (based on telemetry data) and waits until it is greater than 95% """

        prev_remaining_percent = None
        async for battery in self.drone.telemetry.battery():
            if prev_remaining_percent != battery.remaining_percent:
                prev_remaining_percent = battery.remaining_percent
                self.logger.debug(f"Charging... Battery is {battery.remaining_percent:.2f}% charged")

            if battery.remaining_percent > 0.95:
                self.logger.info("Charging complete")
                return

    @transition(
        source=DroneState.FLYING,
        target=DroneState.READY
    )
    async def observe_flying(self):
        """ Monitors whether the drone is flying or not and returns after landing """
        was_in_air = False

        async for is_in_air in self.drone.telemetry.in_air():
            if is_in_air:
                was_in_air = is_in_air

            if was_in_air and not is_in_air:
                return

    async def observe_flight_mode(self):
        prev_mode = None
        async for mode in self.drone.telemetry.flight_mode():
            if mode != prev_mode:
                prev_mode = mode
                self.logger.info(f"Flight mode {mode.name}")

    @transition(
        source=DroneState.READY,
        target=DroneState.FLYING,
    )
    async def monitor_mission_progress(self):
        async for mission_progress in self.drone.mission_raw.mission_progress():
            self.logger.info(f"Mission progress: {mission_progress.current}/{mission_progress.total}")

    async def preflight_checks(self) -> bool:
        async for health in self.drone.telemetry.health():
            if not all(vars(health).values()):
                failed_checks = [p for p, v in vars(health).items() if v is False]
                self.logger.warning(f"PRE-FLIGHT checks FAILED: {failed_checks}")
                return False
            return True

        return True

    @transition(
        source=[DroneState.BOOTSTRAPPING, DroneState.READY, DroneState.CHARGING],
        target=None,
    )
    async def update(self):
        """ Get updates from the ground station """
        self.logger.info("Starting updating...")
        await self.updater.download_next_mission_plan()
        plan = await self.drone.mission_raw.import_qgroundcontrol_mission("data/mission.plan")
        await self.drone.mission_raw.upload_mission(plan.mission_items)
        self.logger.info("Update done...")

    @transition(
        source=DroneState.READY,
        target=DroneState.FLYING
    )
    async def takeoff(self):
        await self.drone.action.arm()
        await self.drone.mission_raw.start_mission()

    @transition(
        source=DroneState.READY,
        target=DroneState.READY
    )
    async def fly(self):
        self.logger.info("Starting the mission...")
        can_start_mission = await self.preflight_checks()
        if not can_start_mission:
            self.logger.warning("Cannot start the mission because of PRE-FLIGHT checks")
            return

        self.add_new_bg_task(self.monitor_mission_progress())
        try:
            await self.takeoff()
            await self.observe_flying()
        except ActionError:
            self.logger.warning("Cannot Start the mission")
        finally:
            await self.terminate_running_tasks()
        self.logger.info("The mission ended")

    @transition(
        source=DroneState.READY,
        target=DroneState.CHARGING
    )
    async def start_charging(self):
        self.logger.info("Start charging")
        await asyncio.sleep(0.2)

    @transition(
        source=DroneState.READY,
        target=DroneState.READY
    )
    async def rest(self):
        """ This is the rest state, where the drone make updates and wait till it's charged """
        self.add_new_bg_task(self.update())
        await self.start_charging()
        await self.observe_charging()
        await self.terminate_running_tasks()

    async def run(self):
        await self.bootstrap()
        self.add_new_bg_task(self.observe_flight_mode())
        while True:
            await self.fly()
            await self.rest()





import asyncio
import logging
import math
import time
from enum import Enum

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.telemetry import Position

from aiofsm import StateMachine, transition

from core.exceptions import ConnectionFailedTooManyTimes
from core.updater import Updater
from helpers.conversions import DEG_2_RAD, RAD_2_DEG
from precision_landing.tracker import SingleMarkerTracker, MarkerAngles, LandingMarker


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
    def __init__(
        self,
        connection_address: str,
        max_retries: int = 3,
        use_visual_landing: bool = False,
        show_visual_landing: bool = False,
        calibration_folder: str = "data/camera/calibration",
    ):
        super().__init__()
        self._connection_address = connection_address
        self.drone = System()
        self.updater = Updater()
        self.logger = logging.getLogger(__name__)
        self.is_simulation = "serial://" not in connection_address
        self.max_connection_retries = max_retries
        self.connection_attempts = 0
        self.running_tasks = []  # Maybe swapping the order of the mixins

        self.use_visual_landing = use_visual_landing
        self.landing_marker_tracker = SingleMarkerTracker(
            0, 14.0, show_frame=show_visual_landing, calibration_folder=calibration_folder,
        )
        self.landing_frequency_input = 1  # Hertz
        self.landing_safe_altitude = 25  # Centimeters below which we enter LAND mode
        self.landing_descend_angle = 20 * DEG_2_RAD
        self.landing_descend_speed = 30.0

    @transition(
        source=DroneState.BOOTSTRAPPING,
        target=DroneState.READY,
        on_error=DroneState.BOOTSTRAPPING,
        exception=ConnectionFailedTooManyTimes,
    )
    async def bootstrap(self, test=False):
        """The boostrap phase is needed to connect the software to the drone via MavLink telemetry.
        This should be the first method to be called"""
        self.logger.info("Bootstrapping...")
        await self.drone.connect(self._connection_address)
        self.logger.debug("Connected to the address. Checking state")

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

        if not test:
            self.logger.info("Setting defaults")
            # await self.drone.action.set_return_to_launch_altitude(10.0)  # It sends the wrong parameter
            # await self.drone.action.set_takeoff_altitude(10.0)  # It sends the wrong parameter

            await self.update()
        self.logger.info("Bootstrapping completed")

    @transition(
        source=DroneState.CHARGING,
        target=DroneState.READY,
    )
    async def observe_charging(self):
        """Monitor the battery percentage (based on telemetry data) and waits until it is greater than 95%"""

        prev_remaining_percent = None
        async for battery in self.drone.telemetry.battery():
            if prev_remaining_percent != battery.remaining_percent:
                prev_remaining_percent = battery.remaining_percent
                self.logger.debug(
                    f"Charging... Battery is {battery.remaining_percent*100:.2f}% charged"
                )

            if battery.remaining_percent > 0.95:
                self.logger.info("Charging complete")
                return

    @transition(source=DroneState.FLYING, target=DroneState.READY)
    async def observe_flying(self):
        """Monitors whether the drone is flying or not and returns after landing"""
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
            self.logger.info(
                f"Mission progress: {mission_progress.current}/{mission_progress.total}"
            )

            if self.use_visual_landing and mission_progress.current == mission_progress.total - 1:
                await self.visual_landing()

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
        """Get updates from the ground station"""
        self.logger.info("Starting updating...")

        plan_path = await self.updater.download_next_mission_plan()
        plan = await self.drone.mission_raw.import_qgroundcontrol_mission(plan_path)
        self.logger.info("Importing Mission")
        await self.drone.mission_raw.upload_mission(plan.mission_items)
        self.logger.info("Update done")

    async def wait_for_global_position(self):
        async for health in self.drone.telemetry.health():
            self.logger.info("Waiting Global Estimate position")
            if health.is_global_position_ok:
                self.logger.info("Global Estimate position is OK, ready to start")
                return

    @transition(source=DroneState.READY, target=DroneState.FLYING)
    async def takeoff(self):
        await self.drone.action.arm()
        await self.drone.action.takeoff()
        await self.drone.mission_raw.start_mission()

    @transition(source=DroneState.READY, target=DroneState.READY)
    async def fly(self):
        self.logger.info("Starting the mission...")
        can_start_mission = await self.preflight_checks()
        if not can_start_mission:
            self.logger.warning("Cannot start the mission because of PRE-FLIGHT checks, waiting for global position")
            await self.wait_for_global_position()

        self.add_new_bg_task(self.monitor_mission_progress())
        try:
            await self.takeoff()
            await self.observe_flying()
        except ActionError as e:
            self.logger.warning("Cannot Start the mission")
            self.logger.error(e)
        finally:
            await self.terminate_running_tasks()
        self.logger.info("The mission ended")

    @transition(source=DroneState.READY, target=DroneState.CHARGING)
    async def start_charging(self):
        self.logger.info("Start charging")
        await asyncio.sleep(0.2)

    @transition(source=DroneState.READY, target=DroneState.READY)
    async def rest(self):
        """This is the rest state, where the drone make updates and wait till it's charged"""
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

    def _should_decrease_altitude(self, marker_angle: MarkerAngles):
        return (
            math.sqrt(marker_angle.angle_x ** 2 + marker_angle.angle_y ** 2)
            <= self.landing_descend_angle
        )

    @staticmethod
    def _get_precise_height(current_position: Position, marker: LandingMarker) -> float:
        return (
            current_position.relative_altitude_m
            if current_position.relative_altitude_m >= 5
            else marker.z
        )

    async def get_current_position(self) -> Position:
        async for position in self.drone.telemetry.position():
            return position

    async def get_current_yaw_deg(self) -> float:
        async for attitude in self.drone.telemetry.attitude_euler():
            return attitude.yaw_deg

    async def get_current_mission_item(self) -> int:
        async for progress in self.drone.mission_raw.mission_progress():
            return progress.current

    async def observe_is_armed(self):  # sourcery skip: use-assigned-variable
        prev_state = None
        async for is_armed in self.drone.telemetry.armed():
            if is_armed != prev_state:
                prev_state = is_armed
                self.logger.info(f"Is Armed? {is_armed}")

            if not is_armed:
                self.use_visual_landing = False

    async def visual_landing(self):
        last_command_input = time.time()
        should_rtl = False
        while self.use_visual_landing:
            current_mission_item = await self.get_current_mission_item()
            marker = self.landing_marker_tracker.track()
            if not marker:
                await asyncio.sleep(0.5)
                continue

            if marker:
                self.logger.info("Landing Marker found, pausing mission.")
                await self.drone.mission_raw.pause_mission()
                should_rtl = True

                current_position = await self.get_current_position()
                current_height = self._get_precise_height(current_position, marker)

                if (
                    time.time()
                    >= last_command_input + 1.0 / self.landing_frequency_input
                ):
                    last_command_input = time.time()

                    marker_angle = marker.estimate_angles_to_marker(current_height)
                    self.logger.info(
                        f"Visual Landing: {marker} -> "
                        f"angle_x={marker_angle.angle_x * RAD_2_DEG} "
                        f"angle_y={marker_angle.angle_y * RAD_2_DEG}"
                    )

                    yaw_deg = await self.get_current_yaw_deg()
                    marker_lat, marker_lon = marker.get_coordinates(
                        current_position, yaw_deg
                    )
                    if self._should_decrease_altitude(marker_angle):
                        absolute_altitude_m = current_position.absolute_altitude_m - (
                            self.landing_descend_speed
                            * 0.01
                            / self.landing_frequency_input
                        )
                    else:
                        absolute_altitude_m = current_position.absolute_altitude_m

                    self.logger.info(
                        f"Commanding to go at "
                        f"LAT={marker_lat} "
                        f"LON={marker_lon} "
                        f"ALT={absolute_altitude_m} "
                        f"YAW={yaw_deg}"
                    )
                    await self.drone.action.goto_location(
                        marker_lat, marker_lon, absolute_altitude_m, yaw_deg
                    )

                if current_height <= self.landing_safe_altitude:
                    self.logger.info("Landing")
                    await self.drone.action.land()
            elif should_rtl:
                await self.drone.mission_raw.set_current_mission_item(current_mission_item)
                should_rtl = False

        self.landing_marker_tracker.stop()
        self.logger.info("Landed")
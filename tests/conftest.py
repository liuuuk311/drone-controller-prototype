import pytest
import cv2 as cv
from core.controller import DroneController
from precision_landing.tracker import SingleMarkerTracker


@pytest.fixture
def mocked_core():
    class MockedCore:
        def __init__(self, system):
            self.system = system

        async def connection_state(self, *args, **kwargs):
            class State:
                def __init__(self, is_connected):
                    self.is_connected = is_connected

            for attempt in range(10):
                if attempt > 2:
                    self.system.tasks_done.append("connection_state_ok")

                yield State(attempt > 2)

    return MockedCore


@pytest.fixture
def mocked_action():
    class MockedAction:
        def __init__(self, system):
            self.system = system

        async def set_takeoff_altitude(self, *args, **kwargs):
            self.system.tasks_done.append("set_takeoff_altitude_ok")

        async def set_return_to_launch_altitude(self, *args, **kwargs):
            self.system.tasks_done.append("set_return_to_launch_altitude_ok")

        async def arm(self, *args, **kwargs):
            self.system.tasks_done.append("arm_ok")

        async def return_to_launch(self, *args, **kwargs):
            self.system.tasks_done.append("return_to_launch_ok")

    return MockedAction


@pytest.fixture
def mocked_mission_raw():
    class MissionRaw:

        def __init__(self, system):
            self.system = system

        async def import_qgroundcontrol_mission(self, *args, **kwargs):
            class Plan:
                mission_items = None

            self.system.tasks_done.append("import_qgroundcontrol_mission_ok")
            return Plan()

        async def upload_mission(self, *args, **kwargs):
            self.system.tasks_done.append("upload_mission_ok")

        async def mission_progress(self, *args, **kwargs):
            class MissionProgress:
                current = 0
                total = 0
            self.system.tasks_done.append("mission_progress_ok")
            for x in range(3):
                yield MissionProgress()

        async def start_mission(self, *args, **kwargs):
            self.system.tasks_done.append("start_mission_ok")

    return MissionRaw


@pytest.fixture
async def mocked_health_not_ok():
    class Health:
        def __init__(self):
            self.is_gyrometer_calibration_ok = False
            self.is_accelerometer_calibration_ok = False
            self.is_magnetometer_calibration_ok = False
            self.is_level_calibration_ok = False
            self.is_local_position_ok = False
            self.is_global_position_ok = False
            self.is_home_position_ok = False

    async def health(*args, **kwargs):
        for x in range(3):
            yield Health()

    return health


@pytest.fixture
def mocked_telemetry():
    class Telemetry:
        def __init__(self, system):
            self.system = system

        async def battery(self, *args, **kwargs):
            class Battery:
                def __init__(self, p):
                    self.remaining_percent = float(p)

            self.system.tasks_done.append("battery_ok")
            for percentage in range(0, 100, 5):
                yield Battery(percentage / 100)

        async def in_air(self, *args, **kwargs):
            """ sequence of False, True, False. Distribution: 10%, 40%, 50%  """
            self.system.tasks_done.append("in_air_ok")
            for x in range(10):
                yield False if x < 10 or x > 50 else True

        async def health(self, *args, **kwargs):
            class Health:
                def __init__(self):
                    self.is_gyrometer_calibration_ok = True
                    self.s_accelerometer_calibration_ok = True
                    self.is_magnetometer_calibration_ok = True
                    self.is_level_calibration_ok = True
                    self.is_local_position_ok = True
                    self.is_global_position_ok = True
                    self.is_home_position_ok = True

            self.system.tasks_done.append("health_ok")
            for x in range(3):
                yield Health()

        async def flight_mode(self, *args, **kwargs):
            class FlightMode:
                def __init__(self):
                    self.name = "STAB"

            self.system.tasks_done.append("flight_mode_ok")
            for percentage in range(3):
                yield FlightMode()

    return Telemetry


@pytest.fixture
def mocked_system(mocked_core, mocked_action, mocked_mission_raw, mocked_telemetry):

    class MockedSystem:
        def __init__(self):
            self.tasks_done = []
            self.action = mocked_action(self)
            self.core = mocked_core(self)
            self.mission_raw = mocked_mission_raw(self)
            self.telemetry = mocked_telemetry(self)

        async def connect(self, *args, **kwargs):
            self.tasks_done.append("connect_ok")
            pass

    return MockedSystem()


@pytest.fixture
def drone_controller(monkeypatch, mocked_system):
    c = DroneController(connection_address="test_address", calibration_folder="../data/camera/test")
    monkeypatch.setattr(c, "drone", mocked_system)
    return c


# ========================= TRACKER =============================

@pytest.fixture
def mocked_cam():
    class Cam:
        def __init__(self, filename):
            self.filename = filename

        def read(self):
            return True, cv.imread(filename=self.filename)

        def release(self):
            pass

    return Cam

import asyncio

import pytest

from aiofsm.exceptions import InvalidStartState
from core.controller import DroneController, DroneStateMachine, DroneState


def test_drone_state_machine():
    fsm = DroneStateMachine()
    assert fsm.is_bootstrapping

    fsm.state = DroneState.READY
    assert fsm.is_ready

    fsm.state = DroneState.FLYING
    assert fsm.is_flying

    fsm.state = DroneState.CHARGING
    assert fsm.is_charging


def test_drone_controller_init():
    # Test __init__ without args
    with pytest.raises(TypeError) as ex_info:
        DroneController()
        assert ex_info.value == "__init__() missing 1 required positional argument: 'connection_address'"

    # Test simulation __init__
    c = DroneController(connection_address="test_address", calibration_folder="../data/camera/test")
    assert c._connection_address == "test_address"

    # Test default attributes
    assert c.is_simulation
    assert c.running_tasks == []
    assert c.is_bootstrapping

    # Test real __init__
    c = DroneController(connection_address="serial:///dev/test", calibration_folder="../data/camera/test")
    assert c.is_simulation is False


@pytest.mark.asyncio
async def test_bootstrap_ok(drone_controller: DroneController):
    assert drone_controller.is_bootstrapping
    await drone_controller.bootstrap()
    assert drone_controller.is_ready

    assert drone_controller.drone.tasks_done == [
        # Init
        "connect_ok",
        "connection_state_ok",
        # Update
        "import_qgroundcontrol_mission_ok",
        "upload_mission_ok"
    ]


@pytest.mark.asyncio
async def test_bootstrap_max_retries_low(drone_controller: DroneController):
    # The connection state will yield True after 2 connection attempts
    drone_controller.max_connection_retries = 1

    assert drone_controller.is_bootstrapping
    await drone_controller.bootstrap()
    assert drone_controller.is_bootstrapping

    assert drone_controller.drone.tasks_done == [
        "connect_ok",
    ]


@pytest.mark.asyncio
async def test_bootstrap_from_wrong_state(drone_controller: DroneController):
    drone_controller.state = DroneState.FLYING
    assert drone_controller.is_flying
    with pytest.raises(InvalidStartState):
        await drone_controller.bootstrap()
    assert drone_controller.is_flying


@pytest.mark.asyncio
async def test_observe_charging(drone_controller: DroneController):
    drone_controller.state = DroneState.CHARGING
    await drone_controller.observe_charging()
    assert drone_controller.is_ready

    assert drone_controller.drone.tasks_done == [
        "battery_ok"
    ]


@pytest.mark.asyncio
async def test_observe_charging_from_wrong_state(drone_controller: DroneController):
    drone_controller.state = DroneState.FLYING
    with pytest.raises(InvalidStartState):
        await drone_controller.observe_charging()
    assert drone_controller.is_flying


@pytest.mark.asyncio
async def test_observe_flying(drone_controller: DroneController):
    drone_controller.state = DroneState.FLYING
    await drone_controller.observe_flying()
    assert drone_controller.is_ready

    assert drone_controller.drone.tasks_done == [
        "in_air_ok"
    ]


@pytest.mark.asyncio
async def test_observe_flying_from_wrong_state(drone_controller: DroneController):
    drone_controller.state = DroneState.BOOTSTRAPPING
    with pytest.raises(InvalidStartState):
        await drone_controller.observe_flying()
    assert drone_controller.is_bootstrapping


@pytest.mark.asyncio
async def test_observe_flight_mode(drone_controller: DroneController):
    await drone_controller.observe_flight_mode()

    assert drone_controller.drone.tasks_done == [
        "flight_mode_ok"
    ]



@pytest.mark.asyncio
async def test_add_new_task(drone_controller: DroneController):
    async def fake_task():
        await asyncio.sleep(0.1)

    assert drone_controller.running_tasks == []
    drone_controller.add_new_bg_task(fake_task())
    assert len(drone_controller.running_tasks) == 1

    await fake_task()


@pytest.mark.asyncio
async def test_terminate_running_tasks(drone_controller: DroneController):
    async def fake_long_task():
        await asyncio.sleep(10)

    assert drone_controller.running_tasks == []
    drone_controller.add_new_bg_task(fake_long_task())
    assert len(drone_controller.running_tasks) == 1

    # terminate
    await drone_controller.terminate_running_tasks()
    assert len(drone_controller.running_tasks) == 0


@pytest.mark.asyncio
async def test_monitor_mission_progress(drone_controller: DroneController):
    drone_controller.state = DroneState.READY
    await drone_controller.monitor_mission_progress()
    assert drone_controller.is_flying

    assert drone_controller.drone.tasks_done == [
        "mission_progress_ok"
    ]


@pytest.mark.asyncio
async def test_preflight_checks(drone_controller: DroneController, mocked_health_not_ok, monkeypatch):
    assert await drone_controller.preflight_checks()

    assert drone_controller.drone.tasks_done == [
        "health_ok"
    ]
    monkeypatch.setattr(drone_controller.drone.telemetry, "health", mocked_health_not_ok)
    assert await drone_controller.preflight_checks() is False


@pytest.mark.asyncio
async def test_update(drone_controller: DroneController):
    old_state = drone_controller.state
    await drone_controller.update()
    assert drone_controller.state == old_state

    assert drone_controller.drone.tasks_done == [
        "import_qgroundcontrol_mission_ok",
        "upload_mission_ok"
    ]


@pytest.mark.asyncio
async def test_update_from_wrong_state(drone_controller: DroneController):
    drone_controller.state = DroneState.FLYING
    with pytest.raises(InvalidStartState):
        await drone_controller.update()
    assert drone_controller.is_flying

    assert drone_controller.drone.tasks_done == []


@pytest.mark.asyncio
async def test_takeoff(drone_controller: DroneController):
    drone_controller.state = DroneState.READY
    await drone_controller.takeoff()
    assert drone_controller.is_flying

    assert drone_controller.drone.tasks_done == [
        "arm_ok",
        "start_mission_ok"
    ]


@pytest.mark.asyncio
async def test_takeoff_from_wrong_state(drone_controller: DroneController):
    drone_controller.state = DroneState.BOOTSTRAPPING
    with pytest.raises(InvalidStartState):
        await drone_controller.takeoff()
    assert drone_controller.is_bootstrapping

    assert drone_controller.drone.tasks_done == []


@pytest.mark.asyncio
async def test_fly(drone_controller: DroneController):
    drone_controller.state = DroneState.READY
    await drone_controller.fly()
    assert drone_controller.is_ready

    assert drone_controller.drone.tasks_done == [
        "health_ok",
        "arm_ok",
        "start_mission_ok",
        "in_air_ok",
    ]


@pytest.mark.asyncio
async def test_fly_from_wrong_state(drone_controller: DroneController):
    drone_controller.state = DroneState.BOOTSTRAPPING
    with pytest.raises(InvalidStartState):
        await drone_controller.fly()
    assert drone_controller.is_bootstrapping



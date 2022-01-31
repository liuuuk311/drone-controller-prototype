import math

import numpy as np
import pytest
from mavsdk.telemetry import Position

from precision_landing.tracker import SingleMarkerTracker, LandingMarker, MarkerAngles


def test_tracker_non_existing_configuration():
    """
        An OSError should be raised is the calibration files are not found
    """
    with pytest.raises(OSError) as ex_info:
        SingleMarkerTracker(0, 14.0, calibration_folder="/path/non/exists", debug=False)
        assert "Cannot load a configuration, /path/non/exists/camera_matrix.data not found" in ex_info


def test_tracker_with_marker(monkeypatch, mocked_cam):
    """
        The track method should return a LandingMarker object if the marker is present
    """
    t = SingleMarkerTracker(0, 14.0, calibration_folder="../data/camera/test", debug=False)
    monkeypatch.setattr(t, "_cam", mocked_cam("../data/camera/test/images/dark_far_center.jpg"))

    result = t.track()
    assert type(result) == LandingMarker


def test_tracker_x_estimation(monkeypatch, mocked_cam):
    """
        The more right the marker is, the greater the x should be
    """
    t = SingleMarkerTracker(0, 14.0, calibration_folder="../data/camera/test", debug=False)
    monkeypatch.setattr(t, "_cam", mocked_cam("../data/camera/test/images/light_close_right.jpg"))
    right_marker = t.track()

    monkeypatch.setattr(t, "_cam", mocked_cam("../data/camera/test/images/light_close_left.jpg"))
    left_marker = t.track()

    assert right_marker.x > left_marker.x


def test_tracker_y_estimation(monkeypatch, mocked_cam):
    """
        When the marker is at the top the Y should be negative. When is at the bottom, Y should be positive
    """
    t = SingleMarkerTracker(0, 14.0, calibration_folder="../data/camera/test", debug=False)
    monkeypatch.setattr(t, "_cam", mocked_cam("../data/camera/test/images/paper_close_top.jpg"))
    top_marker = t.track()

    monkeypatch.setattr(t, "_cam", mocked_cam("../data/camera/test/images/paper_far_bottom.jpg"))
    bottom_marker = t.track()

    assert top_marker.y < bottom_marker.y


def test_tracker_z_estimation(monkeypatch, mocked_cam):
    """
        The more the marker is far from the camera, the greater the z should be
    """
    t = SingleMarkerTracker(0, 14.0, calibration_folder="../data/camera/test", debug=False)
    monkeypatch.setattr(t, "_cam", mocked_cam("../data/camera/test/images/paper_close_top.jpg"))
    close_marker = t.track()

    monkeypatch.setattr(t, "_cam", mocked_cam("../data/camera/test/images/paper_far_bottom.jpg"))
    far_marker = t.track()

    assert close_marker.z < far_marker.z


def test_tracker_without_marker(monkeypatch, mocked_cam):
    """
        The track method should return None if the marker is not present
    """
    t = SingleMarkerTracker(0, 14.0, calibration_folder="../data/camera/test", debug=False)
    monkeypatch.setattr(t, "_cam", mocked_cam("../data/camera/test/images/dark_no_marker.jpg"))

    result = t.track()
    assert result is None


def test_landing_marker():
    lm = LandingMarker(x=0, y=0, z=0, rotations=np.zeros(1), translations=np.zeros(1))

    assert lm.x == 0
    assert lm.y == 0
    assert lm.z == 0
    assert lm.rotations == np.zeros(1)
    assert lm.translations == np.zeros(1)


def test_landing_marker_frame_ref_camera_to_uav():
    lm = LandingMarker(x=10, y=5, z=0, rotations=np.zeros(1), translations=np.zeros(1))

    uav_position = lm.convert_reference_from_camera_to_uav()
    assert uav_position.x == -10
    assert uav_position.y == -5


def test_landing_marker_estimate_angles_to_marker():
    lm = LandingMarker(x=0, y=0, z=0, rotations=np.zeros(1), translations=np.zeros(1))

    marker_angles = lm.estimate_angles_to_marker(uav_height=None)
    assert type(marker_angles) == MarkerAngles

    assert marker_angles.angle_x == 0
    assert marker_angles.angle_y == 0

    lm = LandingMarker(x=1, y=0, z=5, rotations=np.zeros(1), translations=np.zeros(1))
    marker_angles = lm.estimate_angles_to_marker(uav_height=None)
    assert round(marker_angles.angle_x, 2) == -0.2
    assert marker_angles.angle_y == 0

    lm = LandingMarker(x=0, y=1, z=5, rotations=np.zeros(1), translations=np.zeros(1))
    marker_angles = lm.estimate_angles_to_marker(uav_height=None)
    assert marker_angles.angle_x == 0
    assert round(marker_angles.angle_y, 2) == -0.2

    lm = LandingMarker(x=1, y=1, z=1, rotations=np.zeros(1), translations=np.zeros(1))
    marker_angles = lm.estimate_angles_to_marker(uav_height=None)
    assert marker_angles.angle_x == -math.pi/4
    assert marker_angles.angle_y == -math.pi/4

    # uav_height too low, using precise z from landing marker
    lm = LandingMarker(x=1, y=1, z=1, rotations=np.zeros(1), translations=np.zeros(1))
    marker_angles = lm.estimate_angles_to_marker(uav_height=3)
    assert marker_angles.angle_x == -math.pi / 4
    assert marker_angles.angle_y == -math.pi / 4

    # uav_height high, using uav_height
    lm = LandingMarker(x=100, y=100, z=1, rotations=np.zeros(1), translations=np.zeros(1))
    marker_angles = lm.estimate_angles_to_marker(uav_height=10)
    assert round(marker_angles.angle_x, 2) == -0.1
    assert round(marker_angles.angle_y, 2) == -0.1


def test_landing_marker_north_east_coordinates():
    lm = LandingMarker(x=1, y=1, z=1, rotations=np.zeros(1), translations=np.zeros(1))
    north, east = lm.get_north_east_directions_for_uav(0)
    assert north == -1
    assert east == -1

    lm = LandingMarker(x=-1, y=1, z=1, rotations=np.zeros(1), translations=np.zeros(1))
    north, east = lm.get_north_east_directions_for_uav(0)
    assert north == 1
    assert east == -1

    lm = LandingMarker(x=1, y=-1, z=1, rotations=np.zeros(1), translations=np.zeros(1))
    north, east = lm.get_north_east_directions_for_uav(0)
    assert north == -1
    assert east == 1

    lm = LandingMarker(x=-1, y=1, z=1, rotations=np.zeros(1), translations=np.zeros(1))
    north, east = lm.get_north_east_directions_for_uav(315)  # -45 deg
    assert north < 1e-6
    assert round(east, 4) == -1.4142


def test_landing_marker_get_coordinates():
    lm = LandingMarker(x=-1, y=-1, z=1, rotations=np.zeros(1), translations=np.zeros(1))
    original_position = Position(latitude_deg=45, longitude_deg=8, absolute_altitude_m=100, relative_altitude_m=10)

    lat, lon = lm.get_coordinates(original_position, 90)
    assert round(lat, 6) == 44.999991
    assert round(lon, 6) == 8.000013

    lat, lon = lm.get_coordinates(original_position, 0)
    assert round(lat, 6) == 45.000009
    assert round(lon, 6) == 8.000013

    lat, lon = lm.get_coordinates(original_position, 180)
    assert round(lat, 6) == 44.999991
    assert round(lon, 6) == 7.999987

    lat, lon = lm.get_coordinates(original_position, 270)
    assert round(lat, 6) == 45.000009
    assert round(lon, 6) == 7.999987

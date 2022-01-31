import time

from mavsdk.telemetry import Position

if __name__ == '__main__':
    import sys
    import os
    sys.path.insert(1, os.path.join(sys.path[0], '..'))

import argparse
import logging
import math
from collections import namedtuple
from typing import List, Optional, Tuple

from helpers.constants import EARTH_RADIUS
from helpers.conversions import DEG_2_RAD, RAD_2_DEG
import numpy as np
import cv2 as cv
import cv2.aruco as aruco

DetectedMarkers = namedtuple("DetectedMarkers", ["corners", "ids"])
UAVPosition = namedtuple("UAVPosition", ["x", "y"])
MarkerAngles = namedtuple("MakerAngles", ["angle_x", "angle_y"])


class LandingMarker:
    def __init__(self, x: float, y: float, z: float, rotations: np.ndarray, translations: np.ndarray):
        self.x = x
        self.y = y
        self.z = z

        self.rotations = rotations
        self.translations = translations

    def convert_reference_from_camera_to_uav(self) -> UAVPosition:
        """
        Convert the reference frame from camera to UAV.
        Assuming the camera on the UAV is facing down,
        Originally: the camera has X right and Y down, the UAV has X forward and Y right
        Now the camera is mounted sideways, the camera has X backward and Y left
        """

        # return UAVPosition(-self.y, self.x)
        return UAVPosition(-self.x, -self.y)

    def estimate_angles_to_marker(self, uav_height: Optional[float] = None) -> MarkerAngles:
        """
        Estimate the angles of the marker with respect to the center of the UAV.
        Useful to understand if the error is low.

        uav_height is in meter and we need z it in cm

        The angles returned are in radians
        """
        if uav_height and uav_height >= 5:
            z = uav_height * 100.0
        else:
            z = self.z

        uav_position = self.convert_reference_from_camera_to_uav()

        angle_x = math.atan2(uav_position.x, z)
        angle_y = math.atan2(uav_position.y, z)

        return MarkerAngles(angle_x, angle_y)

    def get_north_east_directions_for_uav(self, current_yaw: float) -> Tuple[float, float]:
        c = math.cos(current_yaw * DEG_2_RAD)
        s = math.sin(current_yaw * DEG_2_RAD)

        uav_position = self.convert_reference_from_camera_to_uav()

        north = uav_position.x * c - uav_position.y * s
        east = uav_position.x * s + uav_position.y * c

        return north, east

    def get_coordinates(self, original_position: Position, current_yaw: float) -> Tuple[float, float]:
        """
            Get Lat, Long of the marker relative to the original position and yaw
            The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

            For more information see:
            http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
       """
        north, east = self.get_north_east_directions_for_uav(current_yaw)
        latitude_offset = north / EARTH_RADIUS
        longitude_offset = east / (EARTH_RADIUS * math.cos(original_position.latitude_deg * DEG_2_RAD))

        new_latitude_deg = original_position.latitude_deg + (latitude_offset * RAD_2_DEG)
        new_longitude_deg = original_position.longitude_deg + (longitude_offset * RAD_2_DEG)

        return new_latitude_deg, new_longitude_deg

    def __str__(self):
        return f"LandingMarker(x={self.x}, y={self.y}, z={self.z})"


class SingleMarkerTracker:
    def __init__(
            self,
            marker_id: int,
            marker_size: float,
            camera_size: Optional[List] = None,
            calibration_folder: str = "data/camera/calibration",
            show_frame: bool = False,
            debug: bool = False
            ):

        self.logger = logging.getLogger(__name__)
        self.marker_id = marker_id
        self.marker_size = marker_size

        try:
            self._camera_matrix = np.loadtxt(f"{calibration_folder}/camera_matrix.data")
            self._camera_distortion = np.loadtxt(f"{calibration_folder}/camera_distortion_coef.data")
        except OSError as e:
            self.logger.error(f"Cannot load a configuration, {e}")
            raise OSError(f"Cannot load a configuration, {e}")

        self.logger.info("Read camera matrix and distortion")

        self._stop = False
        self._debug = debug
        self._show = show_frame

        self._aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        self._parameters = aruco.DetectorParameters_create()

        camera_size = [640, 480] if camera_size is None else camera_size
        self._cam = cv.VideoCapture(0)
        self._cam.set(cv.CAP_PROP_FRAME_WIDTH, camera_size[0])
        self._cam.set(cv.CAP_PROP_FRAME_HEIGHT, camera_size[1])

    def _get_landing_marker(self, corners) -> LandingMarker:
        # rotation: Array of Arrays -> attitude of the marker respect to camera frame
        # translation: Array of Arrays -> position of the marker in camera frame
        rotations, translations, _ = aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self._camera_matrix, self._camera_distortion
        )

        # We are dealing just with the first marker
        rotations, translations = rotations[0, 0, :], translations[0, 0, :]
        x, y, z = translations
        return LandingMarker(x, y, z, rotations, translations)

    def _show_detected_marker(self, frame, corners, marker) -> None:
        aruco.drawDetectedMarkers(frame, corners)
        aruco.drawAxis(frame, self._camera_matrix, self._camera_distortion, marker.rotations, marker.translations, 10)

    def stop(self):
        self._stop = True
        self._cam.release()

        if self._show:
            cv.destroyAllWindows()

    def _detect_markers(self, gray_image) -> DetectedMarkers:
        corners, ids, _ = aruco.detectMarkers(
            image=gray_image,
            dictionary=self._aruco_dict,
            parameters=self._parameters,
            cameraMatrix=self._camera_matrix,
            distCoeff=self._camera_distortion
        )
        ids_found = ids.flatten() if ids is not None else None
        return DetectedMarkers(corners, ids_found)

    def is_landing_marker_found(self, markers: DetectedMarkers) -> bool:
        return markers.ids is not None and self.marker_id == markers.ids[0]

    def track(self) -> Optional[LandingMarker]:
        _, frame = self._cam.read()
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        detected_markers = self._detect_markers(gray)
        marker = None
        if self.is_landing_marker_found(detected_markers):
            marker = self._get_landing_marker(detected_markers.corners)

            if self._show:
                self._show_detected_marker(frame, detected_markers.corners, marker)

            if self._debug:
                # self.logger.info(f"MARKER Position x={marker.x:.4f} y={marker.y:.4f} z={marker.z:.4f}")
                uav_ref = marker.convert_reference_from_camera_to_uav()
                self.logger.info(f"Pos from UAV x={uav_ref.x:.4f} y={uav_ref.y:.4f}")
        else:
            if self._debug:
                self.logger.info("Nothing detected.")

        if self._show:
            cv.imshow('Camera', frame)
            cv.waitKey(5)

        return marker


if __name__ == '__main__':
    import sys, os
    sys.path.insert(1, os.path.join(sys.path[0], '..'))

    logFormatter = ' %(asctime)s %(name)s - %(levelname)s: %(message)s'
    logging.basicConfig(level=logging.DEBUG, format=logFormatter)

    parser = argparse.ArgumentParser(description='Track a single Aruco Marker with the camera. '
                                                 'The aruco dictionary used is DICT_4X4_250')
    parser.add_argument(
        "-d", "--display", action="store_true", dest="display", help="Display the frames and the marker overlay")

    args = parser.parse_args()

    finder = SingleMarkerTracker(0, 14.0, show_frame=bool(args.display), debug=True)
    try:
        while True:
            finder.track()
    except KeyboardInterrupt:
        finder.stop()

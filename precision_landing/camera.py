import logging
import time
import glob
import random
import argparse
import numpy as np
import cv2 as cv


class Camera:
    def __init__(self, calibration_path="data/camera/calibration"):
        self.logger = logging.getLogger(__name__)
        self.calibration_folder = calibration_path

    def __enter__(self):
        self.cam = cv.VideoCapture(0)

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cam.release()

    def take_pictures(
            self, num_pictures: int = 10, delay: int = 5, save_for_calibration: bool = True, display: bool = False):
        if not hasattr(self, "cam") or not self.cam:
            self.logger.error("Camera is not initialized. Please this class as context manager")
            raise RuntimeError("Camera is not initialized.")

        picture_taken = 0
        start_time = time.time()
        while picture_taken < num_pictures:
            _, frame = self.cam.read()
            if display:
                cv.imshow("Camera", frame)
                cv.waitKey(5)

            if time.time() - start_time >= delay:
                self.logger.info("Picture taken")
                picture_taken += 1
                if save_for_calibration:
                    path = f"{self.calibration_folder}/images/{picture_taken}.jpg"
                    cv.imwrite(path, frame)
                    self.logger.info(f"Image saved in {path}")

                self.logger.info(f"Waiting {delay}s before next picture")
                start_time = time.time()

        if display:
            cv.destroyAllWindows()

    def calibrate(self, display_results: bool = False):
        termination_criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        chessboard_corner_points = np.zeros((6 * 7, 3), np.float32)
        chessboard_corner_points[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

        real_world_points = []  # 3d point in real world space
        image_points = []  # 2d points in image plane.

        images = glob.glob(f'{self.calibration_folder}/images/*.jpg')
        image_size = None
        for image_filename in images:
            img = cv.imread(image_filename)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            image_size = gray.shape[::-1]
            found, corners = cv.findChessboardCorners(gray, (7, 6), None)

            if found:
                real_world_points.append(chessboard_corner_points)
                image_points.append(corners)

                if display_results:
                    corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), termination_criteria)
                    cv.drawChessboardCorners(img, (7, 6), corners2, found)
                    cv.imshow('img', img)
                    cv.waitKey(500)

        self.logger.info(f"Processed {len(images)} files. "
                         f"Found corners in {len(real_world_points)} images. "
                         f"Success rate {len(real_world_points)/float(len(images))}")
        if display_results:
            cv.destroyAllWindows()

        _, camera_mat, distortion_coefs, rotation_vect, translation_vect = cv.calibrateCamera(
            real_world_points, image_points, image_size, None, None)

        np.savetxt(f'{self.calibration_folder}/camera_matrix.data', camera_mat)
        np.savetxt(f'{self.calibration_folder}/camera_distortion_coef.data', distortion_coefs)
        self.logger.info("Camera Matrix and Distortion coefficients saved")

        if display_results:
            img = cv.imread(random.choice(images))
            height, width = img.shape[:2]
            optimal_camera_mat, roi = cv.getOptimalNewCameraMatrix(
                camera_mat, distortion_coefs, (width, height), 1, (width, height))

            undistorted_img = cv.undistort(img, camera_mat, distortion_coefs, None, optimal_camera_mat)

            # crop the image
            x, y, w, h = roi
            undistorted_img = undistorted_img[y:y + height, x:x + width]
            cv.imshow("Calibration results", undistorted_img)
            cv.waitKey(2500)

            cv.imwrite(f'{self.calibration_folder}/calibresult.png', undistorted_img)


if __name__ == '__main__':
    logFormatter = ' %(asctime)s %(name)s - %(levelname)s: %(message)s'
    logging.basicConfig(level=logging.DEBUG, format=logFormatter)

    parser = argparse.ArgumentParser(description='Use the main camara of the computer.')
    parser.add_argument("-p", "--take-pictures", metavar="N", nargs=1, dest="pictures", help="Take N pictures")
    parser.add_argument("-d", "--display", action="store_true", dest="display", help="Display the frames and results")
    parser.add_argument("-c", "--calibrate", dest="calibrate", action="store_true", help="Calibrate the camera")
    parser.add_argument("-o", "--output", dest="output_path", nargs=1, help="Output folder path")

    args = parser.parse_args()
    if args.output_path:
        cam = Camera(args.output_path[0])
    else:
        cam = Camera()

    if args.pictures:
        with cam:
            cam.take_pictures(
                num_pictures=int(args.pictures[0]),
                delay=2,
                save_for_calibration=True,
                display=bool(args.display)
            )

    if args.calibrate:
        cam.calibrate(display_results=args.display)
        45.76815571525464, 9.089966558123605
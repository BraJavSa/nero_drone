#!/usr/bin/env python3

# Core script implementation.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import glob
import pickle

CHESSBOARD_SIZE = (10, 7)
SQUARE_SIZE = 2.0
CALIBRATION_IMAGES_DIR = 'calibration_images'
OUTPUT_DIR = 'output'
ROS_TOPIC = '/bebop/camera/image_raw'

class ROSCameraCapture(Node):
    def __init__(self):
        super().__init__('ros_camera_capture')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, ROS_TOPIC, self.image_callback, 10)
        self.frame = None
        self.get_logger().info(f"Listening for images on {ROS_TOPIC} ...")
        os.makedirs(CALIBRATION_IMAGES_DIR, exist_ok=True)

    def image_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

def calibrate_camera():
    objp = np.zeros((CHESSBOARD_SIZE[0]*CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    objpoints = []
    imgpoints = []

    images = glob.glob(os.path.join(CALIBRATION_IMAGES_DIR, '*.jpg'))
    if not images:
        print(f"No images found in {CALIBRATION_IMAGES_DIR}")
        return None, None, None, None, None

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print(f"Found {len(images)} images for calibration.")

    for idx, fname in enumerate(images):
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

        if ret:
            objpoints.append(objp)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            cv2.drawChessboardCorners(img, CHESSBOARD_SIZE, corners2, ret)
            cv2.imwrite(os.path.join(OUTPUT_DIR, f'corners_{os.path.basename(fname)}'), img)
            print(f"[{idx+1}/{len(images)}] Pattern detected in {fname}")
        else:
            print(f"[{idx+1}/{len(images)}] Chessboard not detected in {fname}")

    if not objpoints:
        print("No valid chessboard patterns detected.")
        return None, None, None, None, None

    print("Calibrating camera...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None)

    calibration_data = {
        'camera_matrix': mtx,
        'distortion_coefficients': dist,
        'rotation_vectors': rvecs,
        'translation_vectors': tvecs,
        'reprojection_error': ret
    }

    with open(os.path.join(OUTPUT_DIR, 'calibration_data.pkl'), 'wb') as f:
        pickle.dump(calibration_data, f)

    np.savetxt(os.path.join(OUTPUT_DIR, 'camera_matrix.txt'), mtx)
    np.savetxt(os.path.join(OUTPUT_DIR, 'distortion_coefficients.txt'), dist)

    print(f"\nCalibration completed. RMS error: {ret}")
    print(f"Results saved in: {OUTPUT_DIR}\n")

    return ret, mtx, dist, rvecs, tvecs

def undistort_images(mtx, dist):
    images = glob.glob(os.path.join(CALIBRATION_IMAGES_DIR, '*.jpg'))
    if not images:
        print("No images found to undistort.")
        return

    undistorted_dir = os.path.join(OUTPUT_DIR, 'undistorted')
    os.makedirs(undistorted_dir, exist_ok=True)

    for idx, fname in enumerate(images):
        img = cv2.imread(fname)
        h, w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        output_path = os.path.join(undistorted_dir, f'undistorted_{os.path.basename(fname)}')
        cv2.imwrite(output_path, dst)
        print(f"Undistorted image {idx+1}/{len(images)}: {output_path}")

    print(f"Undistorted images saved in {undistorted_dir}")

def main():
    rclpy.init()
    node = ROSCameraCapture()

    print("=== ROS2 CAMERA CAPTURE ===")
    print("Press 's' to save an image, 'q' to exit.\n")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if node.frame is not None:
                cv2.imshow("ROS2 Camera Feed", node.frame)
                key = cv2.waitKey(1) & 0xFF

                if key == ord('s'):
                    img_name = os.path.join(
                        CALIBRATION_IMAGES_DIR,
                        f"img_{len(glob.glob(os.path.join(CALIBRATION_IMAGES_DIR, '*.jpg'))):03d}.jpg"
                    )
                    cv2.imwrite(img_name, node.frame)
                    print(f"Image saved: {img_name}")

                elif key == ord('q'):
                    print("Stopping capture...")
                    break

    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

    print("\n=== STARTING CALIBRATION ===")
    ret, mtx, dist, rvecs, tvecs = calibrate_camera()
    if mtx is not None:
        undistort_images(mtx, dist)
        print("Calibration and undistortion completed successfully.")
    else:
        print("Camera calibration failed.")

if __name__ == '__main__':
    main()

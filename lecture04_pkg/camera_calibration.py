#!/usr/bin/env python3
import cv2
import numpy as np
import glob
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError

# === Checkerboard config ===
CHECKERBOARD = (8, 6)  # (width, height) = inner corners per row, per column
SQUARE_SIZE = 0.025  # In meters (2.5 cm squares)

class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_calibration_node')
        self.image_sub = self.create_subscription(Image, 'image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.objpoints = []
        self.imgpoints = []
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.output_yaml = "./camera_intrinsics.yaml"

    def image_callback(self, msg):
        self.get_logger().debug('Receiving image')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().info(str(e))
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        if ret:
            objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
            objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
            objp *= SQUARE_SIZE

            self.objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
            self.imgpoints.append(corners2)

            cv2.drawChessboardCorners(cv_image, CHECKERBOARD, corners2, ret)
            cv2.imshow('Corners', cv_image)
            cv2.waitKey(1)

            self.get_logger().info(f"Collected {len(self.objpoints)} valid frames")

            if len(self.objpoints) >= 30:
                self.calibrate_and_save(gray.shape[::-1])
                cv2.destroyAllWindows()
                self.destroy_node()
                rclpy.shutdown()

    def calibrate_and_save(self, image_size):
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, image_size, None, None
        )
        self.get_logger().info(f"Calibration successful: {ret}")
        self.get_logger().info(f"Camera matrix:\n{mtx}")
        self.get_logger().info(f"Distortion coefficients:\n{dist}")

        calib_data = {
            'image_width': image_size[0],
            'image_height': image_size[1],
            'camera_matrix': {
                'rows': 3,
                'cols': 3,
                'data': mtx.flatten().tolist()
            },
            'distortion_model': 'plumb_bob',
            'distortion_coefficients': {
                'rows': 1,
                'cols': len(dist.flatten()),
                'data': dist.flatten().tolist()
            },
            'rectification_matrix': {
                'rows': 3,
                'cols': 3,
                'data': [1, 0, 0, 0, 1, 0, 0, 0, 1]
            },
            'projection_matrix': {
                'rows': 3,
                'cols': 4,
                'data': [*mtx.flatten(), 0, 0, 0, 0]
            }
        }

        with open(self.output_yaml, 'w') as f:
            yaml.dump(calib_data, f)

        self.get_logger().info(f"Calibration saved to {self.output_yaml}")

def main(args=None):
    rclpy.init(args=args)

    node = CameraCalibrationNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
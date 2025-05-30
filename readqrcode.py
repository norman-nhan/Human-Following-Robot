#!/usr/bin/env python3
# coding: utf-8
import cv2
import cv2.aruco as aruco
import numpy as np

# Replace this with CameraInfo topic
camera_matrix = np.array([[600, 0, 320],
                          [0, 600, 240],
                          [0,   0,   1]], dtype=np.float64)
dist_coeffs = np.zeros((5, 1))  # Assuming no distortion

# Parameters
marker_length = 0.15  # TODO: calculate marker length, since pose estimation is highly dependent on this parameter.
desired_offset = np.array([0.0, 0.0, -0.5])  # desired_offset: relative goal position in the marker's coordinate frame.
# The desired position in the camera's coordinate frame will be computed by the following
# desired_pos = tvec + rot_mat @ desired_offset

# Set up camera
cap = cv2.VideoCapture(1) # In Mac OS, 1 is for computer's camera, and 0 is for Iphone's camera.

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters() # for tuning parameters in aruco.ArucoDetector

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    detector = aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, marker_length, camera_matrix, dist_coeffs)

        for i, marker_id in enumerate(ids.flatten()):
            rvec, tvec = rvecs[i][0], tvecs[i][0]

            # Get rotation matrix and yaw angle
            rot_mat, _ = cv2.Rodrigues(rvec)
            yaw = np.arctan2(rot_mat[1, 0], rot_mat[0, 0])  # Yaw angle in radians

            # Compute desired target position in camera frame
            desired_pos = tvec + rot_mat @ desired_offset

            # Display info
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            cv2.putText(frame, f"ID: {marker_id}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"tvec: {tvec.round(2)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 255, 200), 1)
            cv2.putText(frame, f"yaw: {np.degrees(yaw):.1f} deg", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 255, 200), 1)
            cv2.putText(frame, f"desired: {desired_pos.round(2)}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 255), 1)

            # print out rotation matrix & yaw angle
            # print(f'rotation_matrix: {rot_mat}')
            # print(f'yaw: {yaw}')

            # send desired_pos to robot's pose topic
            

            break  # Only process first marker
    cv2.imshow("Aruco Navigation", frame)
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
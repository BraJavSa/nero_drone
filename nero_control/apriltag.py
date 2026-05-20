#!/usr/bin/env python3

# Python-based AprilTag detector and 6-DOF pose estimator using pupil_apriltags.
import cv2
import numpy as np
from dt_apriltags import Detector
from scipy.spatial.transform import Rotation as R
import time

matrix_coefficients = np.array([[1296.0, 0, 679.1841],
                                [0, 1297.8, 352.7879],
                                [0, 0, 1]], dtype=np.float32)
distortion_coefficients = np.array([-0.1501, -0.1480, 0, 0, 0], dtype=np.float32)

fx = matrix_coefficients[0, 0]
fy = matrix_coefficients[1, 1]
cx = matrix_coefficients[0, 2]
cy = matrix_coefficients[1, 2]
intrinsics = [fx, fy, cx, cy]

tag_size = 0.12
tag_family = "tag36h11"
cam = cv2.VideoCapture(0)
detector = Detector(
    families=tag_family,
    nthreads=4,
    quad_decimate=2.0,
    refine_edges=True
)

print("Press 'q' to quit.")
total_frames = 0
detected_frames = 0
processing_times = []
frame_count = 0
last_time = time.time()
fps = 0.0

while True:
    ret, frame = cam.read()
    if not ret: continue
    total_frames += 1
    frame_undistorted = cv2.undistort(frame, matrix_coefficients, distortion_coefficients)
    gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)
    frame_count += 1
    now = time.time()
    elapsed = now - last_time
    if elapsed >= 1.0:
        fps = frame_count / elapsed
        print(f"FPS: {fps:.2f}")
        frame_count = 0
        last_time = now
    det_start = time.time()
    tags = detector.detect(gray, estimate_tag_pose=True, camera_params=intrinsics, tag_size=tag_size)
    det_end = time.time()
    if tags:
        detected_frames += 1
        processing_times.append(det_end - det_start)
        for tag in tags:
            if tag.decision_margin < 30: continue
            tag_id = tag.tag_id
            tvec = tag.pose_t
            R_ct = tag.pose_R
            if np.linalg.det(R_ct) < 0: R_ct[:, 2] *= -1
            cam_H_tag = np.eye(4)
            cam_H_tag[:3, :3] = R_ct
            cam_H_tag[:3, 3] = tvec.flatten()
            pos_tag = cam_H_tag[:3, 3]
            r_rot = R.from_matrix(R_ct)
            roll, pitch, yaw = r_rot.as_euler('xyz', degrees=True)
            dist_total = np.linalg.norm(pos_tag)
            print(f"\n--- TAG {tag_id} ---")
            print(f"Position (X,Y,Z) [m]: {pos_tag[0]:.3f}, {pos_tag[1]:.3f}, {pos_tag[2]:.3f}")
            print(f"Euler (Roll,Pitch,Yaw) [deg]: {roll:.2f}, {pitch:.2f}, {yaw:.2f}")
            print(f"Distance: {dist_total:.3f} m")
            corners = tag.corners.astype(int)
            for i in range(4):
                cv2.line(frame_undistorted, tuple(corners[i]), tuple(corners[(i+1) % 4]), (0, 255, 0), 2)
            cv2.putText(frame_undistorted, f"ID:{tag_id}", tuple(corners[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            axis_length = 0.1
            axis_points = np.float32([[0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length]])
            rvec, _ = cv2.Rodrigues(R_ct)
            tvec = tvec.reshape(3, 1)
            imgpts, _ = cv2.projectPoints(axis_points, rvec, tvec, matrix_coefficients, distortion_coefficients)
            imgpts = np.int32(imgpts).reshape(-1, 2)
            cv2.line(frame_undistorted, tuple(imgpts[0]), tuple(imgpts[1]), (255, 0, 0), 3)
            cv2.line(frame_undistorted, tuple(imgpts[0]), tuple(imgpts[2]), (0, 255, 0), 3)
            cv2.line(frame_undistorted, tuple(imgpts[0]), tuple(imgpts[3]), (0, 0, 255), 3)
    cv2.putText(frame_undistorted, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
    cv2.imshow("Camera", frame_undistorted)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cam.release()
cv2.destroyAllWindows()
if processing_times:
    mean_time = np.mean(processing_times)
    min_time = np.min(processing_times)
    max_time = np.max(processing_times)
else:
    mean_time = min_time = max_time = 0

print("\n========= FINAL REPORT =========")
print(f"Total frames: {total_frames}")
print(f"Detected frames: {detected_frames}")
print(f"Detection rate: {detected_frames/total_frames*100:.2f}%")
print(f"Mean detection time: {mean_time:.4f} s")
print(f"Min detection time: {min_time:.4f} s")
print(f"Max detection time: {max_time:.4f} s")
print("================================")

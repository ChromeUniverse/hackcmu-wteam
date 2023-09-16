import cv2 as cv
import numpy as np
import time

cap = cv.VideoCapture(0)

objpoints = []
imgpoints = []

num_corners_x = 7
num_corners_y = 5

# Define the size of each checker in meters
checker_size = 0.999

objp = np.zeros((num_corners_y * num_corners_x, 3), np.float32)
objp[:,:2] = np.mgrid[0:num_corners_x, 0 : num_corners_y].T.reshape(-1,2) * checker_size

# Limit FPS so we don't take too many frames
last_time = time.time()

while True:
    _, frame = cap.read()

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    ret, corners = cv.findChessboardCorners(gray, (num_corners_x, num_corners_y), None, flags=cv.CALIB_CB_NORMALIZE_IMAGE)
    if ret == True:
        current_time = time.time()
        if (current_time - last_time > 0.3):
            imgpoints.append(corners)
            objpoints.append(objp)
            last_time = current_time
            cv.drawChessboardCorners(frame, (num_corners_x, num_corners_y), corners, ret)
    cv.imshow('frame', frame)
    
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("Camera matrix: \n" + str(mtx))
print("Distortion coefficients: \n" + str(dist))
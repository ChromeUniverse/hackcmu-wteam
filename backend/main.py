import cv2 as cv
import numpy as np
import scipy
from scipy.spatial.transform import Rotation as R
from math import *
from serial import Serial 
import time
import keyboard

# Josh's laptop webcam
# extrinsics = np.array([[976.86470655,   0.,             632.03867508],
#                            [0.,             975.33320954,   359.95917485],
#                            [0.,             0.,             1.          ]])
# distortion = np.array([[ 0.09394829],
#                         [-0.40644515],
#                         [-0.0032688],
#                         [-0.00177299],
#                         [0.39540349]])

arduino = Serial(port='/dev/cu.usbserial-21240', baudrate=9600, timeout=.01) 

extrinsics = np.array([[1.48445170e+03, 0.00000000e+00, 6.44154976e+02],
 [0.00000000e+00, 1.47903554e+03, 4.61964133e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
distortion = np.array([[-2.27169551e-02],  [1.61127973e+00], [-5.74827817e-03], [-5.20267777e-04],
  [-5.91752947e+00]])

global thirteen_pose
global eleven_pose

thirteen_pose = [-50,-50,0]
eleven_pose = [-50,-50,0]

detectorParams = cv.aruco.DetectorParameters()
aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_APRILTAG_36h11)
detector = cv.aruco.ArucoDetector(aruco_dict, detectorParams)

tags = {
    7 : [
        # Translation
        np.array([[0], [0], [0]]),
        # Quaternion
        np.array([[0,0,0]])
    ],
    9: [
        # Translation
        np.array([[0], [-20], [0]]),
        # Quaternion
        np.array([[0,0,0]])
    ],
    11: [
        # Translation
        np.array([[0], [0], [0]]),
        # Quaternion
        np.array([[0,0,0]])
    ],
    13: [
        # Translation
        np.array([[0], [0], [0]]),
        # Quaternion
        np.array([[0,0,0]])
    ]
}

def set_motor_speeds(left,right):
    motor_r_direction = '001' if right > 0 else '000'
    motor_l_direction = '001' if left > 0 else '000'
    motor_r_speed = abs(right)
    motor_l_speed = abs(left)    
    
    print(motor_r_direction, motor_l_direction, motor_r_speed, motor_l_speed) # printing the value 

    data_to_send = bytes(motor_r_direction + motor_l_direction + str(motor_r_speed).zfill(3) + str(motor_l_speed).zfill(3), 'ascii')
    print(data_to_send)
    arduino.write(data_to_send)
    time.sleep(0.05) 

    print("finished writing bytes")


def translation_to_point3d(translation):
#   print(translation)
  return np.array([-translation[1,0], -translation[2,0], +translation[0,0]])

def solve_corner_to_object(translation, tagPose):
#   print("Translation: \n", translation)
#   print("Tag pose: \n", tagPose)
  rotation = R.from_euler('zyx', tagPose[1])
  corner_translation = tagPose[0] + rotation.as_matrix()[0] @ translation
#   print("Rotation matrix: \n", rotation.as_matrix()[0])
#   print(corner_translation)
  return translation_to_point3d(corner_translation)

def solve_corners(tagPose):
    return np.array([
        # solve_corner_to_object(np.array([[0],[-6.889764 / 2],[-6.889764 / 2]]), tagPose),
        # solve_corner_to_object(np.array([[0],[+6.889764 / 2],[-6.889764 / 2]]), tagPose),
        # solve_corner_to_object(np.array([[0],[+6.889764 / 2],[+6.889764 / 2]]), tagPose),
        # solve_corner_to_object(np.array([[0],[-6.889764 / 2],[+6.889764 / 2]]), tagPose)

        solve_corner_to_object(np.array([[0],[-6.889764 / 2],[-6.889764 / 2]]), tagPose),
        solve_corner_to_object(np.array([[0],[+6.889764 / 2],[-6.889764 / 2]]), tagPose),
        solve_corner_to_object(np.array([[0],[+6.889764 / 2],[+6.889764 / 2]]), tagPose),
        solve_corner_to_object(np.array([[0],[-6.889764 / 2],[+6.889764 / 2]]), tagPose)


    ])

def update_vision(cap):
    global thirteen_pose
    global eleven_pose

    ret, frame = cap.read()

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    corners, ids, _ = detector.detectMarkers(gray)
    
    dc = {}

    if ids is not None:
        cv.aruco.drawDetectedMarkers(frame, corners, ids)

        print(ids)

        # print(corners)

        for index in range(len(ids)):
            id = ids[index][0]
            # print(corners[index])

            pose = solve_pose(id,corners[index])

            dc[id] = pose
            # # print(pose)
            # # print(corners[index])
            # p0 = (int(corners[index][0][0][0]),int(corners[index][0][0][1]))
            # p1 = (int(corners[index][0][1][0]),int(corners[index][0][1][1]))
            # # print(p0)
            # # print(p1)
            # frame = cv.circle(frame, p0, radius=10, color=(0, 0, 255), thickness=1)
            # frame = cv.circle(frame, p1, radius=10, color=(0, 255, 255), thickness=1)


            # print("corners: \n", corners[index][0])

    print(dc)

    if 7 in dc.keys() and 11 in dc.keys():
        eleven_pose = [dc[7][0]-dc[11][0], dc[7][1]-dc[11][1], dc[11][2]]
        for item in eleven_pose:
            print(item)
    if 7 in dc.keys() and 13 in dc.keys():
        print(dc[13])
        thirteen_pose = [dc[7][0]-dc[13][0], dc[7][1]-dc[13][1], dc[13][2]]
        for item in thirteen_pose:
            print(item)

    cv.imshow('frame', frame)

    if cv.waitKey(1) == ord('q'):
        exit()

def solve_pose(id, corners):

    # print(id)
    # print(tags[id])
    # print(solve_corners(tags[id]))

    _, rvec, tvec = cv.solvePnP(solve_corners(tags[id]),corners[0],extrinsics,distortion,flags=cv.SOLVEPNP_ITERATIVE)

    rotation_matrix, _ = cv.Rodrigues(rvec)
    translation_vector = -np.dot(np.transpose(rotation_matrix), tvec)

    # rot = R.from_rotvec

    rot = cv.Rodrigues(rvec)[0]

    # print(rot)

    newrot = R.from_matrix(rot)
    # print(id, " ----------------")
    # print(newrot.as_euler("XYZ")[2])
    # print(-newrot.as_euler("XYZ")[0])
    # print(-newrot.as_euler("XYZ")[1])
    # # print(translation_vector)
    # # print([[translation_vector[2,0]],[-translation_vector[0,0]],[-translation_vector[1,0]]])
    # print("----------------")
    # print(translation_vector[2,0])
    # print(-translation_vector[0,0])
    # print(-translation_vector[1,0])

    x = -translation_vector[0,0]
    y = -translation_vector[1,0]
    theta = newrot.as_euler("XYZ")[2]

    return [x * cos(theta) - y * sin(theta), x * sin(theta) + y * cos(theta), theta]

def main():
    cap = cv.VideoCapture(0)
    # cap.set(cv.CAP_PROP_AUTO_EXPOSURE, -7)
    cap.set(cv.CAP_PROP_EXPOSURE, -7) 

    if not cap.isOpened():
        print("error opening camera")
        exit()
    time.sleep(1)

    print("----------------------------")

    t0 = time.time()

    while time.time() - t0 < 1:
        update_vision(cap)


    while True:
        update_vision(cap)
        set_motor_speeds(75,75)
        if (thirteen_pose[1] > -45):
            break
        if cv.waitKey(1) == ord('q'):
            break
    set_motor_speeds(0,0)
    while True:
        update_vision(cap)
        set_motor_speeds(-80,80)
        if (thirteen_pose[2] < -pi/2+21*pi/180):
            break
        if cv.waitKey(1) == ord('q'):
            break
    set_motor_speeds(0,0)
    while True:
        update_vision(cap)
        set_motor_speeds(75,75)
        print("13: ", thirteen_pose[0])
        print("11: ", eleven_pose[0])
        if (thirteen_pose[0] > eleven_pose[0] - 2):
            break
        if cv.waitKey(1) == ord('q'):
            break
    set_motor_speeds(0,0)
    while True:
        update_vision(cap)
        set_motor_speeds(80,-80)
        print("13: ", thirteen_pose[0])
        print("11: ", eleven_pose[0])
        if (thirteen_pose[2] > -23*pi/180):
            break
        if cv.waitKey(1) == ord('q'):
            break
    set_motor_speeds(0,0)
    while True:
        update_vision(cap)
        set_motor_speeds(100,100)
        if (thirteen_pose[1] > -15):
            break
        if cv.waitKey(1) == ord('q'):
            break
    set_motor_speeds(0,0)
    while True:
        update_vision(cap)
        set_motor_speeds(-100,-100)
        if (thirteen_pose[1] < -28):
            break
        if cv.waitKey(1) == ord('q'):
            break
    set_motor_speeds(0,0)
    while True:
        update_vision(cap)
        set_motor_speeds(-80,80)
        if (thirteen_pose[2] < -pi/2+24*pi/180):
            break
        if cv.waitKey(1) == ord('q'):
            break
    while True:
        update_vision(cap)
        set_motor_speeds(75,75)
        print("13: ", thirteen_pose[0])
        print("11: ", eleven_pose[0])
        if (thirteen_pose[0] > eleven_pose[0] + 18):
            break
        if cv.waitKey(1) == ord('q'):
            break
    set_motor_speeds(0,0)
    while True:
        update_vision(cap)
        set_motor_speeds(80,-80)
        print("13: ", thirteen_pose[0])
        print("11: ", eleven_pose[0])
        if (thirteen_pose[2] > -24*pi/180):
            break
        if cv.waitKey(1) == ord('q'):
            break
    set_motor_speeds(0,0)
    while True:
        update_vision(cap)
        set_motor_speeds(80,80)
        if (thirteen_pose[1] > -5):
            break
        if cv.waitKey(1) == ord('q'):
            break
    set_motor_speeds(0,0)
    while True:
        update_vision(cap)
        set_motor_speeds(75,-75)
        if (thirteen_pose[2] > pi/2-17*pi/180):
            break
        if cv.waitKey(1) == ord('q'):
            break
    set_motor_speeds(0,0)
    while True:
        update_vision(cap)
        set_motor_speeds(75,75)
        print("13: ", thirteen_pose[0])
        print("11: ", eleven_pose[0])
        if (thirteen_pose[0] < 59):
            break
        if cv.waitKey(1) == ord('q'):
            break
    set_motor_speeds(0,0)

    cv.destroyAllWindows()

def test():
    solve_corner_to_object(np.array([[1],[0],[0]]),[np.array([[0],[0],[0]]),np.array([[0,pi,0]])])
    corners = solve_corners([np.array([[0],[0],[0]]),np.array([[0,0,0]])])
    for corner in corners:
        print(corner)

if __name__ == "__main__":
    main()
    # test()
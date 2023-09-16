import cv2 as cv
import numpy as np
import scipy
from scipy.spatial.transform import Rotation as R
from math import *

extrinsics = np.array([[976.86470655,   0.,             632.03867508],
                           [0.,             975.33320954,   359.95917485],
                           [0.,             0.,             1.          ]])
distortion = np.array([[ 0.09394829],
                        [-0.40644515],
                        [-0.0032688],
                        [-0.00177299],
                        [0.39540349]])


tags = {
    7 : [
        # Translation
        np.array([[0], [0], [0]]),
        # Quaternion
        np.array([[0,0,0]])
    ],
    9: [
        # Translation
        np.array([[0], [0], [0]]),
        # Quaternion
        np.array([[pi,0,0]])
    ],
    11: [
        # Translation
        np.array([[0], [0], [0]]),
        # Quaternion
        np.array([[pi,0,0]])
    ],
    13: [
        # Translation
        np.array([[0], [0], [0]]),
        # Quaternion
        np.array([[pi,0,0]])
    ]
}

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
    print("----------------")
    print(newrot.as_euler("XYZ")[2])
    print(-newrot.as_euler("XYZ")[0])
    print(-newrot.as_euler("XYZ")[1])
    # print(translation_vector)
    # print([[translation_vector[2,0]],[-translation_vector[0,0]],[-translation_vector[1,0]]])
    print("----------------")
    print(translation_vector[2,0])
    print(-translation_vector[0,0])
    print(-translation_vector[1,0])

def main():
    cap = cv.VideoCapture(0)
    # cap.set(cv.CAP_PROP_AUTO_EXPOSURE, -7)
    cap.set(cv.CAP_PROP_EXPOSURE, -7) 

    detectorParams = cv.aruco.DetectorParameters()
    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_APRILTAG_36h11)
    detector = cv.aruco.ArucoDetector(aruco_dict, detectorParams)

    if not cap.isOpened():
        print("error opening camera")
        exit()
    while True:
        ret, frame = cap.read()

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            cv.aruco.drawDetectedMarkers(frame, corners, ids)

            print(ids)

            # print(corners)

            for index in range(len(ids)):
                id = ids[index][0]
                # print(corners[index])

                solve_pose(id,corners[index])
                # print(corners[index])
                p0 = (int(corners[index][0][0][0]),int(corners[index][0][0][1]))
                p1 = (int(corners[index][0][1][0]),int(corners[index][0][1][1]))
                # print(p0)
                # print(p1)
                frame = cv.circle(frame, p0, radius=10, color=(0, 0, 255), thickness=1)
                frame = cv.circle(frame, p1, radius=10, color=(0, 255, 255), thickness=1)


                # print("corners: \n", corners[index][0])

        cv.imshow('frame', frame)

        if cv.waitKey(1) == ord('q'):
            break

    cv.destroyAllWindows()

def test():
    solve_corner_to_object(np.array([[1],[0],[0]]),[np.array([[0],[0],[0]]),np.array([[0,pi,0]])])
    corners = solve_corners([np.array([[0],[0],[0]]),np.array([[0,0,0]])])
    for corner in corners:
        print(corner)

if __name__ == "__main__":
    main()
    # test()
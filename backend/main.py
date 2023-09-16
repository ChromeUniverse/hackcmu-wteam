import cv2 as cv

def main():
    cap = cv.VideoCapture(0)
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
            # Draw detected markers and IDs
            cv.aruco.drawDetectedMarkers(frame, corners, ids)

        cv.imshow('frame', frame)

        if cv.waitKey(1) == ord('q'):
            break

    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
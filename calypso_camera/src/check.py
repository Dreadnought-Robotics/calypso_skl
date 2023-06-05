#! /usr/bin/python3
import cv2
import numpy as np
import time
import math

ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

def distance(ptX1, ptY1, ptX2, ptY2):
    distance = math.sqrt(((ptY1)-(ptY2))**2 + ((ptX1)-(ptX2))**2)

    return distance

def aruco_display(corners, ids, rejected, image):
    markerID = -1
    if len(corners) > 0: 
        
        ids = ids.flatten()
         
        centres = []

        for (markerCorner, markerID) in zip(corners, ids):

            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

            centres.append((cX, cY))


            cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)
            # print(f"ArUco marker ID: {markerID}")
        
        for i in range(len(centres)-1):
            cv2.line(image, centres[i], centres[i+1], (255, 255, 0), 2)
            # cv2.putText(image, str(distance(centres[i][0], centres[i][1], centres[i+1][0], centres[i][1])), ((centres[i][0]+centres[i+1][0])//2, (centres[i][1]+centres[i+1][1])//2), cv2.FONT_HERSHEY_SIMPLEX,
            #             0.5, (0, 255, 0), 2)
            # cv2.putText(image, str(distance(centres[i][0], centres[i][1], centres[i+1][0], centres[i][1])*10/512)+"cm", ((centres[i][0]+centres[i+1][0])//2, (centres[i][1]+centres[i+1][1])//2), cv2.FONT_HERSHEY_SIMPLEX,
            #             0.5, (0, 255, 0), 2)
            print(str(distance(centres[i][0], centres[i][1], centres[i+1][0], centres[i][1])*10/512))

    return image, markerID

path = r'C:\Users\admin\OneDrive\Desktop\Dreadnought\TAC Norway 2023\Aruco Markers\Vid1.mp4'

aruco_type = "DICT_ARUCO_ORIGINAL"

arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])

arucoParams = cv2.aruco.DetectorParameters()

arr = []
# cap = cv2.VideoCapture(path)
cap = cv2.VideoCapture(1)

while cap.isOpened():

    ret, img = cap.read()

    h, w, _ = img.shape

    width = 1000
    height = int(width*(h/w))
    img = cv2.resize(img, (width//2, height//2), interpolation=cv2.INTER_CUBIC)
    
    corners, ids, rejected = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)

    detected_markers, marker_id = aruco_display(corners, ids, rejected, img)

    if marker_id >= 0 and arr.count(marker_id)==0:
        arr.append(marker_id)
        print(f"Detected {marker_id}")

    cv2.imshow("Image", detected_markers)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        print(arr)
                    
        with open("output.txt", "w") as o:
            for line in arr:
                print("{}".format(line), file=o)
    
        cv2.destroyAllWindows()
        cap.release()
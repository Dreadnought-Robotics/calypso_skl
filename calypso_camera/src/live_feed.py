#! /usr/bin/python3

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import math
from cv_bridge import CvBridge

class live:
    def __init__(self) :
        rospy.init_node('camera_feed_viewer')
        rospy.Subscriber('/calypso/lenovo_cam', Image, self.image_callback1)
        rospy.Subscriber('/calypso/bottom_cam', Image, self.image_callback2)
        self.image1 =  Image()
        self.image2 = Image()

        aruco_type = cv2.aruco.DICT_ARUCO_ORIGINAL

        self.arucoDict = cv2.aruco.getPredefinedDictionary(aruco_type)

        self.arucoParams = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)
        self.arr = []

    def distance(self, ptX1, ptY1, ptX2, ptY2):
        distance = math.sqrt(((ptY1)-(ptY2))**2 + ((ptX1)-(ptX2))**2)

        return distance

    def aruco_display(self, corners, ids, rejected, image):
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
            
            # for i in range(len(centres)-1):
            #     cv2.line(image, centres[i], centres[i+1], (255, 255, 0), 2)
            #     # cv2.putText(image, str(distance(centres[i][0], centres[i][1], centres[i+1][0], centres[i][1])), ((centres[i][0]+centres[i+1][0])//2, (centres[i][1]+centres[i+1][1])//2), cv2.FONT_HERSHEY_SIMPLEX,
            #     #             0.5, (0, 255, 0), 2)
            #     # cv2.putText(image, str(distance(centres[i][0], centres[i][1], centres[i+1][0], centres[i][1])*10/512)+"cm", ((centres[i][0]+centres[i+1][0])//2, (centres[i][1]+centres[i+1][1])//2), cv2.FONT_HERSHEY_SIMPLEX,
            #     #             0.5, (0, 255, 0), 2)
            #     print(str(self.distance(centres[i][0], centres[i][1], centres[i+1][0], centres[i][1])*10/512))

        return image, markerID
    def image_callback1(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.image1 = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def image_callback2(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.image2 = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        h, w, _ = self.image2.shape

        width = 1000
        height = int(width*(h/w))
        self.image2 = cv2.resize(self.image2, (width//2, height//2), interpolation=cv2.INTER_CUBIC)
        # print(type(self.image2))
    def subscribe_and_view_feed(self):

        bridge = CvBridge()
        image1_pub = rospy.Publisher("/calypso/front/aruco", Image, queue_size=10)
        image2_pub = rospy.Publisher("/calypso/bottom/aruco", Image, queue_size=10)
        
        while not rospy.is_shutdown():

            # cv2.line(self.image1, (15, 361), (210, 291), (0, 255, 255), 3)
            # cv2.line(self.image1, (628, 361), (457, 291), (0, 255, 255), 3)
            # cv2.line(self.image1, (457, 291), (210, 291), (0, 0, 255), 3)

            # cv2.imshow("Bottom_feed", self.image1)

            corners, ids, rejected = cv2.aruco.detectMarkers(self.image2, self.arucoDict, parameters=self.arucoParams)

            detected_markers, marker_id = self.aruco_display(corners, ids, rejected, self.image2)

            if marker_id > 0 and self.arr.count(marker_id)==0 and marker_id<100:
                self.arr.append(marker_id)
                print(f"Detected {marker_id}")
            image_msg = bridge.cv2_to_imgmsg(detected_markers, encoding="bgr8")
            image1_pub.publish(image_msg)

            corners, ids, rejected = cv2.aruco.detectMarkers(self.image1, self.arucoDict, parameters=self.arucoParams)

            detected_markers, marker_id = self.aruco_display(corners, ids, rejected, self.image1)

            if marker_id > 0 and self.arr.count(marker_id)==0 and marker_id<100:
                self.arr.append(marker_id)
                print(f"Detected {marker_id}")
            image_msg = bridge.cv2_to_imgmsg(detected_markers, encoding="bgr8")
            image2_pub.publish(image_msg)

            cv2.waitKey(1)

        print(self.arr)
                    
        with open("output.txt", "w") as o:
            for line in self.arr:
                print("{}".format(line), file=o)
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        cam = live()
        cam.subscribe_and_view_feed()
    except rospy.ROSInterruptException:
        pass

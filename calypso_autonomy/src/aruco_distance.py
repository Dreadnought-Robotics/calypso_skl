#!/usr/bin/python3

import cv2 as cv
from cv2 import aruco
import numpy as np
import math
import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Quaternion

class arucos:
    def __init__(self):

        rospy.init_node("getArucoDistances")
        self.cam_mat = np.array([[ 446.2533833421062, 0.000000, 340.01603880435033], [0.000000, 444.8253328039625, 247.08417014716247], [0.0, 0.0, 1.0]])
        self.dist_coef = np.array([[0.0030119123110801745, -0.0010383601226439374, 0.002387225773030714, 0.0011598959710507036, 0.000000]])   
        self.MARKER_SIZE = 16.6  # centimeters
        self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.param_markers = aruco.DetectorParameters()
        self.image=0
        self.length=0
        self.breadth=0
        self.diagonal = math.sqrt((self.length*self.length) + (self.breadth*self.breadth))
        self.distances = rospy.Publisher("/calypso/aruco_distances", Quaternion, queue_size=10)
        self.markers = []
        self.x = []
        self.y = []
        self.q = Quaternion

    def image_callback(self, msg):

        np_arr = np.frombuffer(msg.data, np.uint8)
        #self.image = cv.imdecode(np_arr, cv.IMREAD_COLOR)
        self.image=cv.imread('aruco2.png')
        cv.imshow("frame", self.image)
        #cv.destroyAllWindows()
        self.get_distance()
    
    def get_distance(self):
        
        gray_image = cv.cvtColor(self.image, cv.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = aruco.detectMarkers(gray_image, self.marker_dict, parameters=self.param_markers)
        if marker_corners:
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, self.MARKER_SIZE, self.cam_mat, self.dist_coef)
            total_markers = marker_IDs.size
            z_coordinate = 0
            x_coordinate = 0
            y_coordinate = 0
            print(marker_IDs,"   ",marker_IDs.size)
            iterator=[x for x in range (marker_IDs.size)]
            print(marker_corners[0]) 

            for i in range(total_markers):

                corners=marker_corners[i]
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()

                distance = round(np.sqrt(tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2), 2)
                print(distance)

                cv.polylines(self.image, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)

                z_coordinate = z_coordinate + math.sqrt((distance)*(distance) - (self.diagonal/2)*(self.diagonal/2))
                x_coordinate = x_coordinate + round(tVec[i][0][0],1)
                y_coordinate = y_coordinate + round(tVec[i][0][1],1)

            # for ids, corners, i in zip( iterator , marker_corners, total_markers):
            #     cv.polylines(self.image, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
            #     corners = corners.reshape(4, 2)
            #     corners = corners.astype(int)
            #     top_right = corners[0].ravel()
            #     top_left = corners[1].ravel()
            #     bottom_right = corners[2].ravel()
            #     bottom_left = corners[3].ravel()

            # # Calculating the distance
            #     distance = round(np.sqrt(tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2), 2)
            # # Draw the pose of the marker
            #     # point = cv.drawself.imageAxes(self.image, self.cam_mat, self.dist_coef, rVec[i], tVec[i], 4, 4) 
            #     # cv.putText(self.image,f"id: {ids[0]} Dist: {round(distance, 2)}",top_right,cv.FONT_HERSHEY_PLAIN,1.3,(0, 0, 255),2,cv.LINE_AA,)
            #     # cv.putText(self.image,f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",bottom_right,cv.FONT_HERSHEY_PLAIN,1.0,(0, 0, 255),2,cv.LINE_AA)
            #     z_coordinate = z_coordinate + math.sqrt((distance)*(distance) - (self.diagonal/2)*(self.diagonal/2))
            #     x_coordinate = x_coordinate + round(tVec[i][0][0],1)
            #     y_coordinate = y_coordinate + round(tVec[i][0][1],1)
                
            z_coordinate = z_coordinate/float(total_markers)
            x_coordinate = x_coordinate/float(total_markers)
            y_coordinate = y_coordinate/float(total_markers)
            self.q.x = x_coordinate
            self.q.y = y_coordinate
            self.q.z = z_coordinate
            print(x_coordinate)
            print(y_coordinate)
            print(z_coordinate)
            #self.distances.publish(self.q)

    def start(self):
        rospy.Subscriber('/calypso/lenovo_cam', CompressedImage, self.image_callback)
        rospy.spin()

if __name__ == '__main__':
    x = arucos()
    x.start()

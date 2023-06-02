#!/usr/bin/python3

import math
import rospy
from geometry_msgs.msg import Quaternion
from calypso_msgs.msg import gypseas

def center_distance(msg):
    center1 = math.sqrt((msg.x)*(msg.x) - (diagonal/2)*(diagonal/2))
    center2 = math.sqrt((msg.y)*(msg.y) - (diagonal/2)*(diagonal/2))
    center3 = math.sqrt((msg.z)*(msg.z) - (diagonal/2)*(diagonal/2))
    center4 = math.sqrt((msg.w)*(msg.w) - (diagonal/2)*(diagonal/2))
    g.t1 = g.t2 = constant_rpm + (center1+center2+center3+center4)/4
    eldorado.publish(g)

if __name__ == "__main__":
    rospy.init_node("docking station")
    length=0
    breadth=0
    diagonal = math.sqrt((length*length) + (breadth*breadth))
    constant_rpm = 1500
    g = gypseas()
    rospy.Subscriber("/calypso/aruco_distances", Quaternion, center_distance())
    eldorado = rospy.Publisher("/rosetta/gypseas", gypseas, queue_size=10)
    
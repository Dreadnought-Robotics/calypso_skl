#! /usr/bin/python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

rospy.init_node('calypso_bottom', anonymous=True)

image_pub = rospy.Publisher('calypso/bottom_cam', Image, queue_size=10)

bridge = CvBridge()

cap = cv2.VideoCapture(0)  # 0 represents the default camera, change it to a different index if necessary

# Start the video capture loop
while not rospy.is_shutdown():
    ret, frame = cap.read()

    # Convert the OpenCV frame to a ROS image message
    image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

    # Publish the ROS image message
    image_pub.publish(image_msg)

cap.release()
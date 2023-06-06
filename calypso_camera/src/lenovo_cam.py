#! /usr/bin/python3

import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from std_msgs.msg import Int8, Bool
global flag
flag = Bool()
def callback_switch(msg):
    flag.data = msg.data 

rospy.init_node('forward_camera_publisher')

image_pub1 = rospy.Publisher('/calypso/lenovo_cam', CompressedImage, queue_size=10)
image_pub2 = rospy.Publisher('/calypso/bottom_cam', CompressedImage, queue_size=10)
camera_switcher = rospy.Subscriber('/calypso/camera_switch', Int8, callback_switch)

cap1 = cv2.VideoCapture(0)
while not rospy.is_shutdown():
    while flag.data==1:
        ret2, frame2 = cap2.read()
        if ret2:
            _, compressed_data2 = cv2.imencode('.jpg', frame2)
            msg2 = CompressedImage()
            msg2.format = 'jpeg'
            msg2.data = compressed_data2.tobytes()
            image_pub2.publish(msg2)
        if flag.data==0:
            cap2.release()
            cap1 = cv2.VideoCapture(0)
            break

    while flag.data==0:
        ret1, frame1 = cap1.read()
        if ret1:
            _, compressed_data1 = cv2.imencode('.jpg', frame1)
            msg1 = CompressedImage()
            msg1.format = 'jpeg'
            msg1.data = compressed_data1.tobytes()
            image_pub1.publish(msg1)
        if flag.data==1:
            cap1.release()
            cap2 = cv2.VideoCapture(2)
            break

rospy.shutdown()
cv2.destroyAllWindows()
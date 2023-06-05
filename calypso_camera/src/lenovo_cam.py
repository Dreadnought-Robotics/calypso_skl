#! /usr/bin/python3

import rospy
from sensor_msgs.msg import CompressedImage
import cv2

rospy.init_node('forward_camera_publisher')

image_pub = rospy.Publisher('/calypso/lenovo_cam', CompressedImage, queue_size=10)

cap = cv2.VideoCapture(0)

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if ret:
        _, compressed_data = cv2.imencode('.jpg', frame)
        msg = CompressedImage()
        msg.format = 'jpeg'
        msg.data = compressed_data.tobytes()
        image_pub.publish(msg)
        
cv2.destroyAllWindows()

cap.release()
rospy.shutdown()

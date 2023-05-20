#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub_left = rospy.Subscriber("/oakd_lite/left/image_rect", Image, self.callback1)
        self.image_pub_left = rospy.Publisher("/oakd_lite/left/image_compressed", CompressedImage, queue_size=10)
        self.image_sub_right = rospy.Subscriber("/oakd_lite/right/image_rect", Image, self.callback2)
        self.image_pub_right = rospy.Publisher("/oakd_lite/right/image_compressed", CompressedImage, queue_size=10)
        self.image_sub_rgb= rospy.Subscriber("/oakd_lite/rgb/image", Image, self.callback3)
        self.image_pub_rgb = rospy.Publisher("/oakd_lite/rgb/image_compressed", CompressedImage, queue_size=10)

    def callback1(self, data):
        try:
            # Convert the image message to a OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Convert the OpenCV image to a compressed image message
            compressed_image_msg = CompressedImage()
            compressed_image_msg.header.stamp = rospy.Time.now()
            compressed_image_msg.format = "jpeg"
            compressed_image_msg.data = cv2.imencode('.jpg', cv_image)[1].tostring()

            # Publish the compressed image message
            self.image_pub_left.publish(compressed_image_msg)
        except CvBridgeError as e:
            print(e)
    def callback2(self, data):
        try:
            # Convert the image message to a OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Convert the OpenCV image to a compressed image message
            compressed_image_msg = CompressedImage()
            compressed_image_msg.header.stamp = rospy.Time.now()
            compressed_image_msg.format = "jpeg"
            compressed_image_msg.data = cv2.imencode('.jpg', cv_image)[1].tostring()

            # Publish the compressed image message
            self.image_pub_right.publish(compressed_image_msg)
        except CvBridgeError as e:
            print(e)
    def callback3(self, data):
        try:
            # Convert the image message to a OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Convert the OpenCV image to a compressed image message
            compressed_image_msg = CompressedImage()
            compressed_image_msg.header.stamp = rospy.Time.now()
            compressed_image_msg.format = "jpeg"
            compressed_image_msg.data = cv2.imencode('.jpg', cv_image)[1].tostring()

            # Publish the compressed image message
            self.image_pub_rgb.publish(compressed_image_msg)
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    rospy.init_node('image_converter')
    ic = ImageConverter()
    rospy.spin()
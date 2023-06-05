import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class live:
    def __init__(self) :
        rospy.init_node('camera_feed_viewer')
        # rospy.Subscriber('/calypso/oakd_image', CompressedImage, self.image_callback1)
        rospy.Subscriber('/calypso/lenovo_cam', CompressedImage, self.image_callback2)
        rospy.Subscriber('/calypso/bottom_cam', CompressedImage, self.image_callback2)
        self.image1 =  0
        self.image2 = 0

    def image_callback1(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.image1 = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def image_callback2(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.image2 = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def subscribe_and_view_feed(self):
        
        while not rospy.is_shutdown():

            cv2.imshow("Blue_feed", self.image1)
            cv2.imshow("Front_feed", self.image2)
            cv2.waitKey(1)

        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        cam = live()
        cam.subscribe_and_view_feed()
    except rospy.ROSInterruptException:
        pass

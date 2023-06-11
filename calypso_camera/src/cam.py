#! /usr/bin/python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
from std_msgs.msg import Int8

switch_flag = 0

def switch_callback(msg):
    global switch_flag
    switch_flag = msg.data
    print(msg.data)

class camThread(threading.Thread):
    def __init__(self, previewName, thread_flag):
        threading.Thread.__init__(self)
        self.previewName = previewName
        self.thread_flag = thread_flag
    def run(self):
        print("Starting " + self.previewName)
        if self.thread_flag == 0:
            image_pub1 = rospy.Publisher("/calypso/lenovo_cam", Image, queue_size=10)
            image_pub2 = rospy.Publisher("/calypso/bottom_cam", Image, queue_size=10)
            bridge = CvBridge()
            image_msg1 = Image()
            image_msg2 = Image()
            global switch_flag
            while not rospy.is_shutdown():
                if switch_flag==0:
                    cap = cv2.VideoCapture(0)
                    while not rospy.is_shutdown():
                        ret, frame = cap.read()
                        if ret:
                            image_msg1 = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                            image_pub1.publish(image_msg1)
                            image_pub2.publish(image_msg2)
                elif switch_flag==1:
                    cap = cv2.VideoCapture(0)
                    while not rospy.is_shutdown():
                        ret, frame = cap.read()
                        if ret:
                            image_msg2 = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                            image_pub1.publish(image_msg1)
                            image_pub2.publish(image_msg2)
        elif self.thread_flag == 1:
            switch_subscriber = rospy.Subscriber("/calypso/camera_switch", Int8, switch_callback)


# Create two threads as follows
rospy.init_node('calypso_cams', anonymous=True)
thread1 = camThread("Camera", 0)
thread2 = camThread("Switcher",1)
thread1.start()
thread2.start()

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
            global switch_flag
            while not rospy.is_shutdown():
                if switch_flag==0:
                    camPreview(self.previewName, 0, "/calypso/lenovo_cam", 0)
                elif switch_flag==1:
                    camPreview(self.previewName, 2, "/calypso/bottom_cam", 2)
        elif self.thread_flag == 1:
            switch_subscriber = rospy.Subscriber("/calypso/camera_switch", Int8, switch_callback)

def camPreview(previewName, camID, topicName, check):
    global switch_flag
    image_pub = rospy.Publisher(topicName, Image, queue_size=10)

    bridge = CvBridge()

    cap = cv2.VideoCapture(camID)  # 0 represents the default camera, change it to a different index if necessary

    # Start the video capture loop
    while not rospy.is_shutdown():
        ret, frame = cap.read()

        # Convert the OpenCV frame to a ROS image message
        if ret:
            image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            # print(switch_flag)

            # Publish the ROS image message
            image_pub.publish(image_msg)
        
        if switch_flag != check:
            break

    cap.release()

# Create two threads as follows
rospy.init_node('calypso_cams', anonymous=True)
thread1 = camThread("Camera", 0)
thread2 = camThread("Switcher",1)
thread1.start()
thread2.start()

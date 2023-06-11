import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from quaternions import Quaternion

bridge = CvBridge()

def image_callback(ros_image):
    global bridge

    pipeline_heading_publisher = rospy.Publisher("/publish_three_values",Quaternion,queue_size=10)

    rospy.Subscriber("/calypso_sim/bottom_cam",Image, image_callback)

    img = bridge.imgmsg_to_cv2(ros_image)

    pub_quaternions = Quaternion()

    h = img.shape[0]
    w = img.shape[1]
    # img = cv2.resize(img, (h//2, w//2))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

    contours, hierarchy = cv2.findContours(thresh, 
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    mask = np.zeros_like(img)

    for contour in contours:
        contour_area = cv2.contourArea(contour)

        if (contour_area > 500):
            cv2.drawContours(mask, [contour], -1, (0, 255, 255), thickness=cv2.FILLED)
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # Draw a filled circle at the centroid
                cv2.circle(mask, (cX, cY), 5, (0, 0, 255), -1)
    
    mask2 = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    lines = cv2.HoughLines(mask2, rho=1, theta=np.pi / 180, threshold=100)

    x1=0
    x2=0
    y1=0
    y2=0

    for line in lines:
        rho, theta = line[0]

        # Convert polar coordinates to Cartesian coordinates
        a = np.cos(theta) 
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho

        # Calculate the start and end points of the line to draw
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))

        # Draw the line on the mask image
        print(x1, y1, x2, y2)

        cv2.line(mask2, (x1, y1), (x2, y2), (0, 0, 255), 2)
    
    print(x1, x2, y1, y2)
    # Calculate the angle of the detected line
    angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi

    print(angle)  
    cv2.imshow("Image", img)
    cv2.imshow("Pipeline", mask)

    pub_quaternions.x = cX
    pub_quaternions.y = cY
    pub_quaternions.z = 0
    pub_quaternions.w = angle

    pipeline_heading_publisher.publish(pub_quaternions)

    cv2.waitKey(0)



def main():
    rospy.init_node('pipeline_ros', anonymous=True)

    rospy.spin()
 
    # Close down the video stream when done
    cv2.destroyAllWindows()



if __name__=="__main__":
    main()
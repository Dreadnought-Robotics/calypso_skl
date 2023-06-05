#!usr/bin/env python3
import cv2
import depthai
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

def publish_rgb_topic():
    # Create a DepthAI pipeline
    pipeline = depthai.Pipeline()
    bridge = CvBridge()
    # Create a node to enable the RGB camera stream
    rgb_cam = pipeline.createColorCamera()
    rgb_cam.setPreviewSize(1080, 1080)
    rgb_cam.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)
    rgb_cam.setInterleaved(False)
    rgb_cam.setCamId(0)

    # Create an XLinkOut node to publish the RGB image
    xout_rgb = pipeline.createXLinkOut()
    xout_rgb.setStreamName("rgb")

    # Link the RGB camera output to the XLinkOut node
    rgb_cam.preview.link(xout_rgb.input)
    image_publish = rospy.Publisher("/calypso/oakd_image", CompressedImage, queue_size=10)
    # Start the DepthAI pipeline
    with depthai.Device(pipeline) as device:
        # Get the output queue for the RGB image
        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

        while not rospy.is_shutdown():
            # Retrieve the RGB image frame from the output queue
            rgb_data = q_rgb.tryGet()

            if rgb_data is not None:
                # Retrieve the actual RGB frame data
                rgb_frame = rgb_data.getCvFrame()
                _, compressed_data = cv2.imencode('.jpg', rgb_frame)
                msg = CompressedImage()
                msg.format = 'jpeg'
                msg.data = compressed_data.tobytes()               
                 # Publish the RGB image topic
                image_publish.publish(msg)
    
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node("oakdnode")
    publish_rgb_topic()

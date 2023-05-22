#!/usr/bin/env python3

import os
import cv2
import numpy as np
import depthai as dai
import sys
from pathlib import Path
import yaml

import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2,RegionOfInterest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance,TwistWithCovariance
import cv_bridge as opencv_cv_bridge
from cv_bridge.boost.cv_bridge_boost import getCvType
import tf

class CvBridgeError(TypeError):
    """
    This is the error raised by :class:`cv_bridge.CvBridge` methods when they fail.
    """
    pass


class CvBridge(object):
    """
    The CvBridge is an object that converts between OpenCV Images and ROS Image messages.

       .. doctest::
           :options: -ELLIPSIS, +NORMALIZE_WHITESPACE

           >>> import cv2
           >>> import numpy as np
           >>> from cv_bridge import CvBridge
           >>> br = CvBridge()
           >>> dtype, n_channels = br.encoding_as_cvtype2('8UC3')
           >>> im = np.ndarray(shape=(480, 640, n_channels), dtype=dtype)
           >>> msg = br.cv2_to_imgmsg(im)  # Convert the image to a message
           >>> im2 = br.imgmsg_to_cv2(msg) # Convert the message to a new image
           >>> cmprsmsg = br.cv2_to_compressed_imgmsg(im)  # Convert the image to a compress message
           >>> im22 = br.compressed_imgmsg_to_cv2(msg) # Convert the compress message to a new image
           >>> cv2.imwrite("this_was_a_message_briefly.png", im2)

    """

    def __init__(self):
        import cv2
        self.cvtype_to_name = {}
        self.cvdepth_to_numpy_depth = {cv2.CV_8U: 'uint8', cv2.CV_8S: 'int8', cv2.CV_16U: 'uint16',
                                       cv2.CV_16S: 'int16', cv2.CV_32S:'int32', cv2.CV_32F:'float32',
                                       cv2.CV_64F: 'float64'}

        for t in ["8U", "8S", "16U", "16S", "32S", "32F", "64F"]:
            for c in [1, 2, 3, 4]:
                nm = "%sC%d" % (t, c)
                self.cvtype_to_name[getattr(cv2, "CV_%s" % nm)] = nm

        self.numpy_type_to_cvtype = {'uint8': '8U', 'int8': '8S', 'uint16': '16U',
                                        'int16': '16S', 'int32': '32S', 'float32': '32F',
                                        'float64': '64F'}
        self.numpy_type_to_cvtype.update(dict((v, k) for (k, v) in self.numpy_type_to_cvtype.items()))

    def dtype_with_channels_to_cvtype2(self, dtype, n_channels):
        return '%sC%d' % (self.numpy_type_to_cvtype[dtype.name], n_channels)

    def encoding_to_cvtype2(self, encoding):
        try:
            return getCvType(encoding)
        except RuntimeError as e:
            raise CvBridgeError(e)

    def cv2_to_imgmsg(self, cvim,frame_id, encoding = "passthrough"):
        """
        Convert an OpenCV :cpp:type:`cv::Mat` type to a ROS sensor_msgs::Image message.

        :param cvim:      An OpenCV :cpp:type:`cv::Mat`
        :param encoding:  The encoding of the image data, one of the following strings:

           * ``"passthrough"``
           * one of the standard strings in sensor_msgs/image_encodings.h

        :rtype:           A sensor_msgs.msg.Image message
        :raises CvBridgeError: when the ``cvim`` has a type that is incompatible with ``encoding``

        If encoding is ``"passthrough"``, then the message has the same encoding as the image's OpenCV type.
        Otherwise desired_encoding must be one of the standard image encodings

        This function returns a sensor_msgs::Image message on success, or raises :exc:`cv_bridge.CvBridgeError` on failure.
        """
        if not isinstance(cvim, (np.ndarray, np.generic)):
            raise TypeError('Your input type is not a numpy array')
        img_msg = Image()
        img_msg.header.stamp = rospy.Time.now() # or rospy.get_rostime()
        img_msg.header.frame_id = frame_id

        img_msg.height = cvim.shape[0]
        img_msg.width = cvim.shape[1]
        if len(cvim.shape) < 3:
            cv_type = self.dtype_with_channels_to_cvtype2(cvim.dtype, 1)
        else:
            cv_type = self.dtype_with_channels_to_cvtype2(cvim.dtype, cvim.shape[2])
        if encoding == "passthrough":
            img_msg.encoding = cv_type
        else:
            img_msg.encoding = encoding
            # Verify that the supplied encoding is compatible with the type of the OpenCV image
            if self.cvtype_to_name[self.encoding_to_cvtype2(encoding)] != cv_type:
                raise CvBridgeError("encoding specified as %s, but image has incompatible type %s" % (encoding, cv_type))
        if cvim.dtype.byteorder == '>':
            img_msg.is_bigendian = True
        img_msg.data = cvim.tostring()
        img_msg.step = len(img_msg.data) // img_msg.height

        return img_msg

class oakd_lite():
    def __init__(self):
        # ros config
        #self.bridge = opencv_cv_bridge.CvBridge()
        self.bridge = CvBridge()
        rospy.init_node('oakd_lite')
        self.rgb_pub = rospy.Publisher("/oakd_lite/rgb/image", Image, queue_size=1)
        self.depth_pub = rospy.Publisher("/oakd_lite/depth/image", Image, queue_size=1)
        self.left_pub = rospy.Publisher("/oakd_lite/left/image_rect", Image, queue_size=1)
        self.right_pub = rospy.Publisher("/oakd_lite/right/image_rect", Image, queue_size=1)
        self.left_info_pub = rospy.Publisher("/oakd_lite/left/camera_info", CameraInfo, queue_size=1)
        self.right_info_pub = rospy.Publisher("/oakd_lite/right/camera_info", CameraInfo, queue_size=1)
        self.rgb_info_pub = rospy.Publisher("/oakd_lite/rgb/camera_info", CameraInfo, queue_size=1)
        self.depth_info_pub = rospy.Publisher("/oakd_lite/depth/camera_info", CameraInfo, queue_size=1)
        self.loop_rate = rospy.Rate(30)
        
        # camera calibration method
        self.calibration_method = "image_pipeline" # kalibr, image_pipeline

        # oakd_lite config
        syncNN = True
        # for d455 baseline is 9.5 cm
        self.left_right_baseline = 0.075 # 7.5 cm from official website 

        # Create pipeline
        self.pipeline = dai.Pipeline()
        # Define sources and outputs
        camRgb = self.pipeline.create(dai.node.ColorCamera)
        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        monoRight = self.pipeline.create(dai.node.MonoCamera)
        stereo = self.pipeline.create(dai.node.StereoDepth)

        xoutRgb = self.pipeline.create(dai.node.XLinkOut)
        xoutDepth = self.pipeline.create(dai.node.XLinkOut)
        xout_rectif_left  = self.pipeline.create(dai.node.XLinkOut)
        xout_rectif_right = self.pipeline.create(dai.node.XLinkOut)

        xoutRgb.setStreamName("rgb")
        xoutDepth.setStreamName("depth")
        xout_rectif_left.setStreamName('rectified_left')
        xout_rectif_right.setStreamName('rectified_right')

        # Properties
        #camRgb.setPreviewSize(416, 416)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRgb.setIspScale(1, 3) # (1,3)=640x360 origin from 1920x1080
        self.rgb_width, self.rgb_height = camRgb.getIspSize()

        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P) # THE_400_P, THE_480_P
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        # setting node configs
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        self.depth_width = monoLeft.getResolutionWidth()
        self.depth_height = monoLeft.getResolutionHeight()
        # Align depth map to the perspective of RGB camera, on which inference is done
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        #stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())

        # Linking
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)
        camRgb.isp.link(xoutRgb.input)
        stereo.depth.link(xoutDepth.input)
        stereo.rectifiedLeft.link(xout_rectif_left.input)
        stereo.rectifiedRight.link(xout_rectif_right.input)

    def left_camera_info(self,D,K,R,P,distortion_model,frame_id):
        roi = RegionOfInterest()
        self.left_info = CameraInfo()
        self.left_info.header.stamp = rospy.Time.now() # or rospy.get_rostime()
        self.left_info.header.frame_id = frame_id
        self.left_info.height = self.depth_height
        self.left_info.width = self.depth_width
        self.left_info.distortion_model = distortion_model
        self.left_info.D = D
        self.left_info.K = K
        self.left_info.R = R
        self.left_info.P = P
        self.left_info.binning_x = 0
        self.left_info.binning_y = 0
        self.left_info.roi = roi

    def right_camera_info(self,D,K,R,P,distortion_model,frame_id):
        roi = RegionOfInterest()
        self.right_info = CameraInfo()
        self.right_info.header.stamp = rospy.Time.now() # or rospy.get_rostime()
        self.right_info.header.frame_id = frame_id
        self.right_info.height = self.depth_height
        self.right_info.width = self.depth_width
        self.right_info.distortion_model = distortion_model
        self.right_info.D = D
        self.right_info.K = K
        self.right_info.R = R
        self.right_info.P = P
        self.right_info.binning_x = 0
        self.right_info.binning_y = 0
        self.right_info.roi = roi
        
    def rgb_camera_info(self,D,K,R,P,distortion_model,frame_id):
        roi = RegionOfInterest()
        self.rgb_info = CameraInfo()
        self.rgb_info.header.stamp = rospy.Time.now() # or rospy.get_rostime()
        self.rgb_info.header.frame_id = frame_id
        self.rgb_info.height = self.rgb_height
        self.rgb_info.width = self.rgb_width
        self.rgb_info.distortion_model = distortion_model
        self.rgb_info.D = D
        self.rgb_info.K = K
        self.rgb_info.R = R
        self.rgb_info.P = P
        self.rgb_info.binning_x = 0
        self.rgb_info.binning_y = 0
        self.rgb_info.roi = roi

    def depth_camera_info(self,D,K,R,P,distortion_model,frame_id):
        roi = RegionOfInterest()
        self.depth_info = CameraInfo()
        self.depth_info.header.stamp = rospy.Time.now() # or rospy.get_rostime()
        self.depth_info.header.frame_id = frame_id
        self.depth_info.height = self.rgb_height
        self.depth_info.width = self.rgb_width
        self.depth_info.distortion_model = distortion_model
        self.depth_info.D = D
        self.depth_info.K = K
        self.depth_info.R = R
        self.depth_info.P = P
        self.depth_info.binning_x = 0
        self.depth_info.binning_y = 0
        self.depth_info.roi = roi

    def publish_tf(self):
        br = tf.TransformBroadcaster()
        br.sendTransform((0.0, 0.0, 0.0),
                        (-0.5, 0.5, -0.5, 0.5),
                        rospy.Time.now(),
                        "oakd_depth_camera_optical_frame",
                        "oakd_frame")
        br.sendTransform((0.0, 0.0, 0.0),
                        (-0.5, 0.5, -0.5, 0.5),
                        rospy.Time.now(),
                        "oakd_rgb_camera_optical_frame",
                        "oakd_frame")
        br.sendTransform((0.0, -0.0375, 0.0),
                        (-0.5, 0.5, -0.5, 0.5),
                        rospy.Time.now(),
                        "oakd_right_camera_optical_frame",
                        "oakd_frame")
        br.sendTransform((0.0, 0.0375, 0.0),
                        (-0.5, 0.5, -0.5, 0.5),
                        rospy.Time.now(),
                        "oakd_left_camera_optical_frame",
                        "oakd_frame")

    def main(self):
        # Connect to device and start self.pipeline
        with dai.Device(self.pipeline, usb2Mode = True) as device:
            home_path = os.path.expanduser("~")
            yaml_path = "/vins_ws/src/calypso-Odometry/oakd_development/oakd_lite/oakd_node/config/"
            yaml_name = 'oakd_lite_'+self.calibration_method+'.yaml'
            #with open('./oakd_lite_'+self.calibration_method+'.yaml', 'r') as file:
            with open(home_path+yaml_path+yaml_name, 'r') as file:
                camera_parameter = yaml.safe_load(file)
                rgb_distortion_parameters = camera_parameter["rgb"]["distortion_parameters"]
                rgb_projection_parameters = camera_parameter["rgb"]["projection_parameters"]
                depth_distortion_parameters = camera_parameter["depth"]["distortion_parameters"]
                depth_projection_parameters = camera_parameter["depth"]["projection_parameters"]
                left_distortion_parameters = camera_parameter["left"]["distortion_parameters"]
                left_projection_parameters = camera_parameter["left"]["projection_parameters"]
                right_distortion_parameters = camera_parameter["right"]["distortion_parameters"]
                right_projection_parameters = camera_parameter["right"]["projection_parameters"]

                rgb_D = [rgb_distortion_parameters["k1"],rgb_distortion_parameters["k2"],rgb_distortion_parameters["p1"],rgb_distortion_parameters["p2"],rgb_distortion_parameters["k3"]]
                rgb_K = [rgb_projection_parameters["fx"],0.0,rgb_projection_parameters["cx"],0.0,rgb_projection_parameters["fy"],rgb_projection_parameters["cy"],0.0,0.0,1]
                rgb_R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                rgb_P = [rgb_projection_parameters["fx"],0.0,rgb_projection_parameters["cx"],rgb_projection_parameters["Tx"],0.0,rgb_projection_parameters["fy"],rgb_projection_parameters["cy"],rgb_projection_parameters["Ty"],0.0,0.0,1.0,0.0]

                depth_D = [depth_distortion_parameters["k1"],depth_distortion_parameters["k2"],depth_distortion_parameters["p1"],depth_distortion_parameters["p2"],depth_distortion_parameters["k3"]]
                depth_K = [depth_projection_parameters["fx"],0.0,depth_projection_parameters["cx"],0.0,depth_projection_parameters["fy"],depth_projection_parameters["cy"],0.0,0.0,1]
                depth_R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                depth_P = [depth_projection_parameters["fx"],0.0,depth_projection_parameters["cx"],depth_projection_parameters["Tx"],0.0,depth_projection_parameters["fy"],depth_projection_parameters["cy"],depth_projection_parameters["Ty"],0.0,0.0,1.0,0.0]

                left_D = [left_distortion_parameters["k1"],left_distortion_parameters["k2"],left_distortion_parameters["p1"],left_distortion_parameters["p2"],left_distortion_parameters["k3"]]
                left_K = [left_projection_parameters["fx"],0.0,left_projection_parameters["cx"],0.0,left_projection_parameters["fy"],left_projection_parameters["cy"],0.0,0.0,1]
                left_R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                left_P = [left_projection_parameters["fx"],0.0,left_projection_parameters["cx"],left_projection_parameters["Tx"],0.0,left_projection_parameters["fy"],left_projection_parameters["cy"],left_projection_parameters["Ty"],0.0,0.0,1.0,0.0]

                right_D = [right_distortion_parameters["k1"],right_distortion_parameters["k2"],right_distortion_parameters["p1"],right_distortion_parameters["p2"],right_distortion_parameters["k3"]]
                right_K = [right_projection_parameters["fx"],0.0,right_projection_parameters["cx"],0.0,right_projection_parameters["fy"],right_projection_parameters["cy"],0.0,0.0,1]
                right_R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                right_P = [right_projection_parameters["fx"],0.0,right_projection_parameters["cx"],right_projection_parameters["Tx"],0.0,right_projection_parameters["fy"],right_projection_parameters["cy"],right_projection_parameters["Ty"],0.0,0.0,1.0,0.0]

                rgb_distortion_model = camera_parameter["rgb"]["distortion_model"]
                depth_distortion_model = camera_parameter["depth"]["distortion_model"]
                left_distortion_model = camera_parameter["left"]["distortion_model"]
                right_distortion_model = camera_parameter["right"]["distortion_model"]

            # Output queues will be used to get the rgb frames and nn data from the outputs defined above
            previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            rectified_left_camera_Queue = device.getOutputQueue(name="rectified_left", maxSize=4, blocking=False)
            rectified_right_camera_Queue = device.getOutputQueue(name="rectified_right", maxSize=4, blocking=False)

            while not rospy.is_shutdown():
                self.publish_tf()
                inPreview = previewQueue.get()
                depth = depthQueue.get()
                rectified_left = rectified_left_camera_Queue.get()
                rectified_right = rectified_right_camera_Queue.get()

                frame = inPreview.getCvFrame()
                rectified_left_image = rectified_left.getCvFrame()
                rectified_right_image = rectified_right.getCvFrame()
                depthFrame = depth.getFrame() # depthFrame values are in millimeters

                depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
                depthFrameColor = cv2.equalizeHist(depthFrameColor)
                depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT) # COLORMAP_HOT or COLORMAP_JET

                self.rgb_pub.publish(self.bridge.cv2_to_imgmsg(frame,frame_id="oakd_rgb_camera_optical_frame",encoding="bgr8"))
                self.depth_pub.publish(self.bridge.cv2_to_imgmsg(depthFrame,frame_id="oakd_depth_camera_optical_frame",encoding="mono16")) # depthFrameColor,depthFrame
                self.left_pub.publish(self.bridge.cv2_to_imgmsg(rectified_left_image,frame_id="oakd_left_camera_optical_frame",encoding="mono8"))
                self.right_pub.publish(self.bridge.cv2_to_imgmsg(rectified_right_image,frame_id="oakd_right_camera_optical_frame",encoding="mono8"))

                self.left_camera_info(left_D,left_K,left_R,left_P,rgb_distortion_model,frame_id="oakd_left_camera_optical_frame")
                self.right_camera_info(right_D,right_K,right_R,right_P,depth_distortion_model,frame_id="oakd_right_camera_optical_frame")
                self.rgb_camera_info(rgb_D,rgb_K,rgb_R,rgb_P,left_distortion_model,frame_id="oakd_rgb_camera_optical_frame")
                self.depth_camera_info(depth_D,depth_K,depth_R,depth_P,right_distortion_model,frame_id="oakd_depth_camera_optical_frame")
                self.left_info_pub.publish(self.left_info)
                self.right_info_pub.publish(self.right_info)
                self.rgb_info_pub.publish(self.rgb_info)
                self.depth_info_pub.publish(self.depth_info)
                self.loop_rate.sleep()

if __name__ == '__main__':
    depth_camera = oakd_lite()
    depth_camera.main()
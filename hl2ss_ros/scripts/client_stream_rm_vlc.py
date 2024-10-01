#!/usr/bin/env python3

#------------------------------------------------------------------------------
# This script receives video from one of the HoloLens sideview grayscale
# cameras and publishes it as a ROS 1 node. The camera resolution is 640x480 @ 30 FPS.
# The stream supports three operating modes: 0) video, 1) video + rig pose, 2) query 
# calibration (single transfer).
#------------------------------------------------------------------------------

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped

import cv2
import hl2ss
import hl2ss_lnm

class HoloLensVLCNode:
    def __init__(self):
        rospy.init_node('hololens_vlc_node', anonymous=True)
        
        # ROS Parameters
        self.host = rospy.get_param('~host', '192.168.1.7')
        self.port = rospy.get_param('~port', hl2ss.StreamPort.RM_VLC_LEFTFRONT)
        self.mode = rospy.get_param('~mode', hl2ss.StreamMode.MODE_1)
        self.divisor = rospy.get_param('~divisor', 1)
        self.profile = rospy.get_param('~profile', hl2ss.VideoProfile.H265_MAIN)
        self.bitrate = rospy.get_param('~bitrate', None)

        # Publishers
        self.image_pub = rospy.Publisher('hololens/vlc_image', Image, queue_size=10)
        self.pose_pub = rospy.Publisher('hololens/vlc_pose', PoseStamped, queue_size=10)

        self.bridge = CvBridge()
        self.client = None

    def start(self):
        if self.mode == hl2ss.StreamMode.MODE_2:
            self.query_calibration()
            return

        self.client = hl2ss_lnm.rx_rm_vlc(self.host, self.port, mode=self.mode, divisor=self.divisor, profile=self.profile)
        self.client.open()

        rate = rospy.Rate(30)  # Assuming 30 FPS, adjust if needed
        while not rospy.is_shutdown():
            self.process_frame()
            rate.sleep()

        self.client.close()

    def process_frame(self):
        data = self.client.get_next_packet()

        # Publish image
        img_msg = self.bridge.cv2_to_imgmsg(data.payload.image, encoding="mono8")
        img_msg.header.stamp = rospy.Time.now()
        self.image_pub.publish(img_msg)

        # Publish pose if available
        if self.mode == hl2ss.StreamMode.MODE_1:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            # Convert data.pose to pose_msg.pose
            # This conversion depends on the format of data.pose
            # You may need to adjust this part based on the actual data structure
            self.pose_pub.publish(pose_msg)

        rospy.loginfo(f'Frame captured at {data.timestamp}')
        rospy.loginfo(f'Sensor Ticks: {data.payload.sensor_ticks}')
        rospy.loginfo(f'Exposure: {data.payload.exposure}')
        rospy.loginfo(f'Gain: {data.payload.gain}')

    def query_calibration(self):
        data = hl2ss_lnm.download_calibration_rm_vlc(self.host, self.port)
        rospy.loginfo('Calibration data')
        rospy.loginfo('Image point to unit plane')
        rospy.loginfo(data.uv2xy)
        rospy.loginfo('Extrinsics')
        rospy.loginfo(data.extrinsics)
        rospy.loginfo('Undistort map')
        rospy.loginfo(data.undistort_map)
        rospy.loginfo('Intrinsics (undistorted only)')
        rospy.loginfo(data.intrinsics)

if __name__ == '__main__':
    try:
        node = HoloLensVLCNode()
        node.start()
    except rospy.ROSInterruptException:
        pass

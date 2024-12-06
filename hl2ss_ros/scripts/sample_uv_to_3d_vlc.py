#!/usr/bin/env python3

#------------------------------------------------------------------------------
# This script receives video from one of the HoloLens sideview grayscale
# cameras and publishes it as a ROS 1 node. The camera resolution is 640x480 @ 30 FPS.
# The stream supports three operating modes: 0) video, 1) video + rig pose, 2) query 
# calibration (single transfer).
#------------------------------------------------------------------------------

import sys
import os

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PointStamped
from tf.transformations import quaternion_from_matrix
import cv2
import numpy as np

from viewer import hl2ss
from viewer import hl2ss_lnm
from viewer import hl2ss_3dcv
from enum import Enum

def unity_to_ros_translation(translation):
    return [translation[2], -translation[0], translation[1]]

def unity_to_ros_quaternion(quaternion):
    return [-quaternion[2], quaternion[0], -quaternion[1], quaternion[3]]

def ros_to_unity_translation(translation):
    return [-translation[1], translation[2], translation[0]]

def ros_to_unity_quaternion(quaternion):
    return [quaternion[1], -quaternion[2], -quaternion[0], quaternion[3]]

class HoloLensVLCNode:
    def __init__(self):
        rospy.init_node('sample_uv_to_3d_vlc', anonymous=True)
        self.ag_n = rospy.get_param('~ag_n', '0')
        
        # ROS Parameters
        self.host = rospy.get_param('~host', '192.168.11.33')
        self.port = rospy.get_param('~port', hl2ss.StreamPort.RM_VLC_LEFTFRONT)
        self.mode = rospy.get_param('~mode', hl2ss.StreamMode.MODE_1)
        self.divisor = rospy.get_param('~divisor', 1)
        self.profile = rospy.get_param('~profile', hl2ss.VideoProfile.H264_BASE)
        self.bitrate = rospy.get_param('~bitrate', None)
        self.calibration_path = rospy.get_param('~calibration_path', '../calibration')
        
        # Get calibration data
        self.calibration_vlc = hl2ss_3dcv.get_calibration_rm(self.host, self.port, self.calibration_path)

        # Publishers
        self.image_pub = rospy.Publisher(f'hololens_ag{self.ag_n}/{hl2ss.StreamPort.Name(self.port)}/vlc_image', Image, queue_size=10)
        self.pose_pub = rospy.Publisher(f'hololens_ag{self.ag_n}/{hl2ss.StreamPort.Name(self.port)}/vlc_pose', PoseStamped, queue_size=10)
        self.point_pub_wrist = rospy.Publisher(f'hololens_ag{self.ag_n}/{hl2ss.StreamPort.Name(self.port)}/vlc_point_wrist', PointStamped, queue_size=10)
        self.point_pub_elbow = rospy.Publisher(f'hololens_ag{self.ag_n}/{hl2ss.StreamPort.Name(self.port)}/vlc_point_elbow', PointStamped, queue_size=10)
        
        self.bridge = CvBridge()
        self.client = None
        self.log_info = False

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
        # Get next VLC image
        data = self.client.get_next_packet()

        # Rotate the image 90 degrees clockwise
        rotated_image = cv2.rotate(data.payload.image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    
        # Publish image
        img_msg = self.bridge.cv2_to_imgmsg(rotated_image, encoding="mono8")
        img_msg.header.stamp = rospy.Time.now()
        self.image_pub.publish(img_msg)

        # Publish pose if available
        if self.mode == hl2ss.StreamMode.MODE_1:
            
            # Compute UV to 3D points -------------------------------------------------
            uv_points = np.array([[320, 240], [400, 300]])
            depth_points = np.array([1.0, 1.5])
            
            for i in range(uv_points.shape[0]):
                points_world = self.uv_to_3d(uv_points[i, 0], 
                                           uv_points[i, 1], 
                                           depth_points[i], 
                                           self.calibration_vlc.intrinsics, 
                                           self.calibration_vlc.extrinsics, 
                                           data.pose)
                print(f"World Point for [{uv_points[i, 0]}, {uv_points[i, 1]}, {depth_points[i]}]: {points_world}")
            
            # Get camera transform relative to world frame in column-major order
            camera_to_world = data.pose.transpose()
            
            # Convert to ros coordinate system
            # Unity:   Left-handed:  X=Right,   Y=Up,     Z=Forward
            # ROS:     Right-handed: X=Forward, Y=Left,   Z=Up
            translation = unity_to_ros_translation(camera_to_world[:3, 3])
            rot_se3 = np.eye(4)
            rot_se3[:3, :3] = camera_to_world[:3, :3]
            orientation = unity_to_ros_quaternion(quaternion_from_matrix(rot_se3))
            
            self.publish_pose(translation, orientation, 'world', self.pose_pub)

        if self.log_info == False:
            rospy.loginfo(f'Publishing VLC stream to topic "hololens_ag{self.ag_n}/vlc_image" ...')
            self.log_info = True
        # rospy.loginfo(f'Sensor Ticks: {data.payload.sensor_ticks}')
        # rospy.loginfo(f'Exposure: {data.payload.exposure}')
        # rospy.loginfo(f'Gain: {data.payload.gain}')

    def publish_point(self, point, frame_id, point_pub):
        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = frame_id
        point_msg.point.x = point[0]
        point_msg.point.y = point[1]
        point_msg.point.z = point[2]
        point_pub.publish(point_msg)
        
    def publish_pose(self, translation, orientation, frame_id, pose_pub):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = translation[0]
        pose_msg.pose.position.y = translation[1]
        pose_msg.pose.position.z = translation[2]
        pose_msg.pose.orientation.x = orientation[0]
        pose_msg.pose.orientation.y = orientation[1]
        pose_msg.pose.orientation.z = orientation[2]
        pose_msg.pose.orientation.w = orientation[3]
        pose_pub.publish(pose_msg)
        
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
        
    # Convert UV coordinates to 3D points ----------------------------------------
    def uv_to_3d(self, u, v, depth, intrinsics, extrinsics, pose):
        # Normalize coordinates
        x = (u - intrinsics[2, 0]) / intrinsics[0, 0]
        y = (v - intrinsics[2, 1]) / intrinsics[1, 1]
        
        # Create ray
        xy1 = np.array([x, y, 1])
        
        # Scale by depth
        ray_length = np.sqrt(np.sum(xy1*xy1))
        point_camera = xy1 * (depth / ray_length)
        
        # Transform to world
        # camera_to_world = pose @ hl2ss_3dcv.rignode_to_camera(extrinsics)
        camera_to_world = hl2ss_3dcv.camera_to_rignode(extrinsics) @ hl2ss_3dcv.reference_to_world(pose)
        point_world = hl2ss_3dcv.transform(point_camera, camera_to_world)
        
        return point_world

if __name__ == '__main__':
    try:
        node = HoloLensVLCNode()
        node.start()
    except rospy.ROSInterruptException:
        pass

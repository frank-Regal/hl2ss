#!/usr/bin/env python3

"""
HoloLens VLC Camera Node with UV to 3D Point Conversion

This node receives video from one of the HoloLens sideview grayscale cameras (VLC) 
and performs UV to 3D point conversion. It publishes:
- Camera images (640x480 @ 30 FPS)
- Camera pose 
- 3D points converted from UV coordinates

The stream supports three operating modes:
0) video only
1) video + rig pose
2) query calibration (single transfer)
"""

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

# Coordinate system conversion functions
def unity_to_ros_translation(translation):
    return [translation[2], -translation[0], translation[1]]

def unity_to_ros_quaternion(quaternion):
    return [-quaternion[2], quaternion[0], -quaternion[1], quaternion[3]]

def ros_to_unity_translation(translation):
    return [-translation[1], translation[2], translation[0]]

def ros_to_unity_quaternion(quaternion):
    return [quaternion[1], -quaternion[2], -quaternion[0], quaternion[3]]

class HoloLensVLCNode:
    # -----------------------------------------------------------------------------
    # Initialize the node
    # -----------------------------------------------------------------------------
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('sample_uv_to_3d_vlc', anonymous=True)
        self.ag_n = rospy.get_param('~ag_n', '0')
        
        # Load ROS parameters
        self.host = rospy.get_param('~host', '192.168.11.33')
        self.port = rospy.get_param('~port', hl2ss.StreamPort.RM_VLC_LEFTFRONT)
        self.mode = rospy.get_param('~mode', hl2ss.StreamMode.MODE_1)
        self.divisor = rospy.get_param('~divisor', 1)
        self.profile = rospy.get_param('~profile', hl2ss.VideoProfile.H264_BASE)
        self.bitrate = rospy.get_param('~bitrate', None)
        self.calibration_path = rospy.get_param('~calibration_path', '../calibration')
        
        # Load camera calibration data
        self.calibration_vlc = hl2ss_3dcv.get_calibration_rm(self.host, self.port, self.calibration_path)

        # Set up ROS publishers
        self.port_name = hl2ss.get_port_name(self.port)
        self.image_pub = rospy.Publisher(f'hololens_ag{self.ag_n}/{self.port_name}/image', Image, queue_size=10)
        self.pose_pub = rospy.Publisher(f'hololens_ag{self.ag_n}/{self.port_name}/pose', PoseStamped, queue_size=10)
        self.point_pubs = {
            'wrist': rospy.Publisher(f'hololens_ag{self.ag_n}/{self.port_name}/point_wrist', PointStamped, queue_size=10),
            'elbow': rospy.Publisher(f'hololens_ag{self.ag_n}/{self.port_name}/point_elbow', PointStamped, queue_size=10)
        }
        
        # Initialize ROS bridge and client
        self.bridge = CvBridge()
        self.client = None
        self.log_info = False
        
        # Initialize tracking points
        self.uv_coords = {
            'elbow': [0, 0],  # UV coordinates for elbow
            'wrist': [0, 0]   # UV coordinates for wrist
        }
        
        self.uv_depths = {
            'elbow': 0.0,  # Estimated depth for elbow in meters
            'wrist': 0.0   # Estimated depth for wrist in meters
        }

    # -----------------------------------------------------------------------------
    # Start the VLC stream and ros node
    # -----------------------------------------------------------------------------
    def start(self):
        # Handle calibration query mode
        if self.mode == hl2ss.StreamMode.MODE_2:
            self.query_calibration()
            return

        # Start video stream
        self.client = hl2ss_lnm.rx_rm_vlc(self.host, self.port, mode=self.mode, divisor=self.divisor, profile=self.profile)
        self.client.open()

        # Main processing loop
        rate = rospy.Rate(30)  # 30 FPS processing rate
        while not rospy.is_shutdown():
            self.process_frame()
            rate.sleep()

        self.client.close()

    # -----------------------------------------------------------------------------
    # Process a single frame from the VLC camera
    # -----------------------------------------------------------------------------
    def process_frame(self):
        # Get next frame from VLC camera
        data = self.client.get_next_packet()

        # Rotate image for correct orientation
        rotated_image = cv2.rotate(data.payload.image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    
        # Publish camera image
        img_msg = self.bridge.cv2_to_imgmsg(rotated_image, encoding="mono8")
        img_msg.header.stamp = rospy.Time.now()
        self.image_pub.publish(img_msg)

        # Process pose and points if in MODE_1
        if self.mode == hl2ss.StreamMode.MODE_1:
            
            # Get current UV coordinates and depths
            self.uv_coords['elbow'] = self.get_elbow_point()   
            self.uv_coords['wrist'] = self.get_wrist_point()   
            self.uv_depths['elbow'] = self.get_estimated_depth('elbow')
            self.uv_depths['wrist'] = self.get_estimated_depth('wrist')
            
            # Convert and publish 3D points
            for frame in ['elbow', 'wrist']:
                world_point = self.uv_to_3d(self.uv_coords[frame][0],
                                          self.uv_coords[frame][1],
                                          self.uv_depths[frame],
                                          self.calibration_vlc.intrinsics,
                                          self.calibration_vlc.extrinsics,
                                          data.pose)
                self.publish_point(world_point, 'world', self.point_pubs[frame])
                print(f"World Point for {frame} {self.uv_coords[frame]}, {self.uv_depths[frame]}: {world_point}")
            
            # Process and publish camera pose
            camera_to_world = data.pose.transpose()
            
            # Convert coordinate systems:
            # Unity (Left-handed):  X=Right, Y=Up, Z=Forward
            # ROS (Right-handed):   X=Forward, Y=Left, Z=Up
            translation = unity_to_ros_translation(camera_to_world[:3, 3])
            rot_se3 = np.eye(4)
            rot_se3[:3, :3] = camera_to_world[:3, :3]
            orientation = unity_to_ros_quaternion(quaternion_from_matrix(rot_se3))
            
            self.publish_pose(translation, orientation, 'world', self.pose_pub)

        # Log startup message once
        if self.log_info == False:
            rospy.loginfo(f'Publishing VLC stream to topic "hololens_ag{self.ag_n}/vlc_image" ...')
            self.log_info = True

    # -----------------------------------------------------------------------------
    # Get UV coordinates for wrist and elbow points
    # ----------------------------------------------------------------------------- 
    def get_wrist_point(self):
        return [400, 300]
    
    # -----------------------------------------------------------------------------
    # Get UV coordinates for elbow point
    # -----------------------------------------------------------------------------
    def get_elbow_point(self):
        return [320, 240]
    
    # -----------------------------------------------------------------------------
    # Get estimated depth for a given point in meters
    # -----------------------------------------------------------------------------
    def get_estimated_depth(self, point_name):
        if point_name == 'elbow':
            return 1.0  # Estimated depth for elbow
        elif point_name == 'wrist':
            return 1.5  # Estimated depth for wrist
        else:
            return 1.0  # Default depth
    
    # -----------------------------------------------------------------------------
    # Publish a 3D point as a PointStamped message
    # -----------------------------------------------------------------------------
    def publish_point(self, point, frame_id, point_pub):
        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = frame_id
        point_msg.point.x = point[0]
        point_msg.point.y = point[1]
        point_msg.point.z = point[2]
        point_pub.publish(point_msg)
        
    # -----------------------------------------------------------------------------
    # Publish camera pose as a PoseStamped message
    # -----------------------------------------------------------------------------
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
        
    # -----------------------------------------------------------------------------
    # Query and log camera calibration data
    # -----------------------------------------------------------------------------
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
        
    # -----------------------------------------------------------------------------
    # Convert UV image coordinates to 3D world coordinates.
    # -----------------------------------------------------------------------------
    """
    Args:
        u (float): U coordinate in image space
        v (float): V coordinate in image space
        depth (float): Depth value in meters
        intrinsics (np.ndarray): Camera intrinsic matrix (3x3)
        extrinsics (np.ndarray): Camera extrinsic matrix (4x4)
        pose (np.ndarray): Camera pose matrix (4x4)
    
    Returns:
        np.ndarray: 3D point in world coordinates
    
    Raises:
        ValueError: If depth is zero or negative
    """
    def uv_to_3d(self, u, v, depth, intrinsics, extrinsics, pose):
        if depth <= 0:
            raise ValueError("Depth must be positive")
        
        # Convert image coordinates to normalized coordinates
        x = (u - intrinsics[2, 0]) / intrinsics[0, 0]
        y = (v - intrinsics[2, 1]) / intrinsics[1, 1]
        
        # Create ray vector
        xy1 = np.array([x, y, 1])
        
        # Scale ray by depth
        ray_length = np.sqrt(np.sum(xy1*xy1))
        point_camera = xy1 * (depth / ray_length)
        
        # Transform point from camera to world coordinates
        camera_to_world = hl2ss_3dcv.camera_to_rignode(extrinsics) @ hl2ss_3dcv.reference_to_world(pose)
        point_world = hl2ss_3dcv.transform(point_camera, camera_to_world)
        
        return point_world

if __name__ == '__main__':
    try:
        node = HoloLensVLCNode()
        node.start()
    except rospy.ROSInterruptException:
        pass
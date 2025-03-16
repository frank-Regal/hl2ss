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
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from tf.transformations import quaternion_from_matrix
from geometry_msgs.msg import TransformStamped

import cv2
import numpy as np
from viewer import hl2ss
from viewer import hl2ss_lnm
from viewer import hl2ss_3dcv
from enum import Enum

class HoloLensVLCNode:
    # -----------------------------------------------------------------------------
    # Initialize Node
    # -----------------------------------------------------------------------------
    def __init__(self):
        rospy.init_node('hololens_vlc_node', anonymous=True)

        # ROS Parameters
        self.ag_n =       rospy.get_param('~ag_n'      , '0'                               )
        self.base_frame = rospy.get_param('~base_frame', 'world'                           )
        self.host =       rospy.get_param('~host'      , '192.168.11.33'                   )
        self.port =       rospy.get_param('~port'      , hl2ss.StreamPort.RM_VLC_RIGHTRIGHT)
        self.mode =       rospy.get_param('~mode'      , hl2ss.StreamMode.MODE_1           )
        self.divisor =    rospy.get_param('~divisor'   , 1                                 )
        self.profile =    rospy.get_param('~profile'   , hl2ss.VideoProfile.H264_MAIN      )
        self.bitrate =    rospy.get_param('~bitrate'   , None                              )

        # Publishers
        self.image_pub = rospy.Publisher(f'hololens_ag{self.ag_n}/vlc_image', Image, queue_size=10)
        self.tf_pub =    rospy.Publisher(f'/tf', TFMessage, queue_size=10)

        # Init class variables
        self.tf_msg = TFMessage()
        self.bridge = CvBridge()
        self.frame_id = 'rignode'
        self.client = None
        self.log_info = False
        self.publish_rate = 1 # 10hz
    # -----------------------------------------------------------------------------
    # Start Stream
    # -----------------------------------------------------------------------------
    def start(self):
        # Initialize client
        self.client = hl2ss_lnm.rx_rm_vlc(self.host, self.port, mode=self.mode, divisor=self.divisor, profile=self.profile)
        self.client.open()

        # Start processing frames
        rate = rospy.Rate(30)  # Assuming 30 FPS, adjust if needed
        while not rospy.is_shutdown():
            self.process_frame()
            rate.sleep()

        # Close client
        self.client.close()

    # -----------------------------------------------------------------------------
    # Process Frame
    # -----------------------------------------------------------------------------
    def process_frame(self):
        try:
            data = self.client.get_next_packet()

            # Manipulate image
            rotated_image = cv2.rotate(data.payload.image, hl2ss_3dcv.rm_vlc_get_rotation(self.port))

            # Publish image
            img_msg = self.bridge.cv2_to_imgmsg(rotated_image, encoding="mono8")
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id =  'vlc'

            # Publish tf_msg at 4Hz
            # current_time = rospy.Time.now()
            # if not hasattr(self, 'last_tf_pub_time') or \
            #     (current_time - self.last_tf_pub_time).to_sec() >= self.publish_rate:  # 1/4 = 0.25 seconds for 4hz
            #     self.image_pub.publish(img_msg)
            #     self.last_tf_pub_time = current_time

            self.image_pub.publish(img_msg)


            # Publish /tf if desired
            if self.mode == hl2ss.StreamMode.MODE_1:

                # Convert data.pose to pose_msg.pose
                rignode_to_world = data.pose.transpose() # transpose to convert to column-major order
                rignode_to_world_t, rignode_to_world_r = self.convert_to_ros(rignode_to_world)

                # Append to tf_msg
                self.append_tf_msg(rignode_to_world_t,
                                rignode_to_world_r,
                                self.base_frame,
                                self.frame_id,
                                rospy.Time.now())

                # Publish tf_msg
                self.tf_pub.publish(self.tf_msg)
                self.tf_msg.transforms.clear()

            # Log info if configured
            if self.log_info == False:
                rospy.loginfo(f'Publishing VLC stream to topic "hololens_ag{self.ag_n}/vlc_image" ...')
                self.log_info = True
        except Exception as e:
            rospy.logerr(e)
            pass

    # Convert HL2SS transform to ROS transform
    # -----------------------------------------------------------------------------
    def convert_to_ros(self, transform):
        # Handle translation
        translation = self.hl2ss_to_ros_translation(transform[:3, 3]) # convert from HL2SS to ROS convention

        # Handle rotation
        rotation = self.hl2ss_to_ros_rotation(transform[:3, :3]) # make quaternion and convert to ROS convention

        # Rotate about axis
        rotation = self.rotate_about_axis(rotation, -90, axis='x', frame='body')
        rotation = self.rotate_about_axis(rotation, 180, axis='z', frame='body')

        # rotation = self.rotate_about_axis(rotation, -90, axis='y', frame='body')
        # rotation = self.rotate_about_axis(rotation, 180, axis='z', frame='body')

        # Create 4x4 matrix
        r = np.eye(4)
        r[:3, :3] = rotation
        orientation = quaternion_from_matrix(r)
        return translation, orientation

    # -----------------------------------------------------------------------------
    # Append to tf_msg
    # -----------------------------------------------------------------------------
    def append_tf_msg(self, translation, orientation, frame_id, child_frame_id, timestamp=None):
        tf_stamped = TransformStamped()
        tf_stamped.header.stamp = timestamp
        tf_stamped.header.frame_id = frame_id      # parent frame
        tf_stamped.child_frame_id = child_frame_id # child frame
        tf_stamped.transform.translation.x = translation[0]
        tf_stamped.transform.translation.y = translation[1]
        tf_stamped.transform.translation.z = translation[2]
        tf_stamped.transform.rotation.x = orientation[0]
        tf_stamped.transform.rotation.y = orientation[1]
        tf_stamped.transform.rotation.z = orientation[2]
        tf_stamped.transform.rotation.w = orientation[3]
        self.tf_msg.transforms.append(tf_stamped)

    # -----------------------------------------------------------------------------
    # Convert HL2SS translation to ROS translation
    #   HL2SS (Right-handed): X=right, Y=up, Z=-forward
    #   ROS (Right-handed):   X=forward, Y=left, Z=up
    # -----------------------------------------------------------------------------
    def hl2ss_to_ros_translation(self, translation):
        return [-translation[2], -translation[0], translation[1]]

    # -----------------------------------------------------------------------------
    # Convert HL2SS rotation matrix to ROS rotation matrix
    #   HL2SS (Right-handed): X=right, Y=up, Z=-forward
    #   ROS (Right-handed):   X=forward, Y=left, Z=up
    # -----------------------------------------------------------------------------
    def hl2ss_to_ros_rotation(self, rotation):
        R = np.array([
            [0, 0, -1],
            [-1, 0, 0],
            [0, 1, 0]
        ])
        return R @ rotation @ R.T

    # -----------------------------------------------------------------------------
    # Rotate a matrix about a given axis
    # -----------------------------------------------------------------------------
    def rotate_about_axis(self, matrix_S03, angle_deg, axis=None, frame='space'):
        if axis is None:
            return matrix_S03
        elif axis == 'x':
            rotation_matrix = np.array([[1, 0, 0],
                                      [0, np.cos(np.radians(angle_deg)), -np.sin(np.radians(angle_deg))],
                                      [0, np.sin(np.radians(angle_deg)), np.cos(np.radians(angle_deg))]])
        elif axis == 'y':
            rotation_matrix = np.array([[np.cos(np.radians(angle_deg)), 0, np.sin(np.radians(angle_deg))],
                                      [0, 1, 0],
                                      [-np.sin(np.radians(angle_deg)), 0, np.cos(np.radians(angle_deg))]])
        elif axis == 'z':
            rotation_matrix = np.array([[np.cos(np.radians(angle_deg)), -np.sin(np.radians(angle_deg)), 0],
                                      [np.sin(np.radians(angle_deg)), np.cos(np.radians(angle_deg)), 0],
                                      [0, 0, 1]])
        else:
            raise ValueError(f"Invalid axis: {axis}. Please choose 'x', 'y', or 'z'.")

        if frame == 'space':
            return rotation_matrix @ matrix_S03
        elif frame == 'body':
            return matrix_S03 @ rotation_matrix
        else:
            raise ValueError(f"Invalid frame: {frame}. Please choose 'space' or 'body'.")

    # -----------------------------------------------------------------------------
    # Query Calibration
    # -----------------------------------------------------------------------------
    def query_calibration(self):
        data = hl2ss_lnm.download_calibration_rm_vlc(self.host, self.port)
        rospy.loginfo('Calibration data')
        rospy.loginfo('Image point to unit plane')
        rospy.loginfo(data.uv2xy)
        rospy.loginfo('Extrinsics')
        rospy.loginfo(data.extrinsics)
        self.extrinsics = data.extrinsics
        rospy.loginfo('Undistort map')
        rospy.loginfo(data.undistort_map)
        rospy.loginfo('Intrinsics (undistorted only)')
        rospy.loginfo(data.intrinsics)
        quit()

if __name__ == '__main__':
    try:
        node = HoloLensVLCNode()
        if node.mode == hl2ss.StreamMode.MODE_2:
            node.query_calibration()
        node.start()
    except rospy.ROSInterruptException:
        pass

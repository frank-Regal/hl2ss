#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_matrix
import numpy as np

from viewer import hl2ss
from viewer import hl2ss_lnm

class HoloLensSINode:
    def __init__(self):
        rospy.init_node('hololens_si_node', anonymous=True)
        
        # ROS Parameters
        self.host = rospy.get_param('~host', '192.168.11.33')
        
        # Publishers
        self.head_pose_pub = rospy.Publisher('hololens/head_pose', PoseStamped, queue_size=10)
        
        self.client = None

    def start(self):
        self.client = hl2ss_lnm.rx_si(self.host, hl2ss.StreamPort.SPATIAL_INPUT)
        self.client.open()

        rate = rospy.Rate(30)  # 30 Hz sample rate
        while not rospy.is_shutdown():
            self.process_data()
            rate.sleep()

        self.client.close()

    def process_data(self):
        data = self.client.get_next_packet()
        si = hl2ss.unpack_si(data.payload)

        if si.is_valid_head_pose():
            head_pose = si.get_head_pose()
            
            # print(f'Head pose: Position={head_pose.position} Forward={head_pose.forward} Up={head_pose.up}')
            
            # Create PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "world" 

            ## Convert to right-handed ros coordinate system
            pose_msg.pose.position.x = -head_pose.position[2]
            pose_msg.pose.position.y = -head_pose.position[0]
            pose_msg.pose.position.z = head_pose.position[1]

            # # Rearranged axes to fix rotation
            forward = -np.array(head_pose.forward)  # -z axis
            up = np.array(head_pose.up)            # y axis
            right = np.cross(up, forward)          # x axis

            # Create rotation matrix
            rot = np.array([right, up, forward]).T
            rotmat = np.eye(4)
            rotmat[:3, :3] = rot
            
            # Convert to quaternion [x, y, z, w]
            quaternion = quaternion_from_matrix(rotmat)

            # Set orientation                    
            pose_msg.pose.orientation.w = -quaternion[3]
            pose_msg.pose.orientation.x = quaternion[2]
            pose_msg.pose.orientation.y = quaternion[0]
            pose_msg.pose.orientation.z = -quaternion[1]
            
            # Publish the pose
            self.head_pose_pub.publish(pose_msg)
            rospy.loginfo(f'Head pose published at {rospy.Time.now()}')

if __name__ == '__main__':
    try:
        node = HoloLensSINode()
        node.start()
    except rospy.ROSInterruptException:
        pass
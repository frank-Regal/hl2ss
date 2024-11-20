#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
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
            
            # Create PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "world"  # or whatever your reference frame is

            # Set position
            pose_msg.pose.position.x = head_pose.position[0]
            pose_msg.pose.position.y = head_pose.position[1]
            pose_msg.pose.position.z = head_pose.position[2]

            # Convert forward and up vectors to quaternion
            # Note: right = cross(up, -forward)
            forward = -np.array(head_pose.forward)  # -z axis
            up = np.array(head_pose.up)            # y axis
            right = np.cross(up, forward)          # x axis

            # Create rotation matrix
            rotation_matrix = np.array([right, up, forward]).T
            
            # Convert rotation matrix to quaternion
            # Using a simplified conversion assuming the matrix is orthogonal
            trace = rotation_matrix.trace()
            if trace > 0:
                S = np.sqrt(trace + 1.0) * 2
                w = 0.25 * S
                x = (rotation_matrix[2,1] - rotation_matrix[1,2]) / S
                y = (rotation_matrix[0,2] - rotation_matrix[2,0]) / S
                z = (rotation_matrix[1,0] - rotation_matrix[0,1]) / S
            else:
                if rotation_matrix[0,0] > rotation_matrix[1,1] and rotation_matrix[0,0] > rotation_matrix[2,2]:
                    S = np.sqrt(1.0 + rotation_matrix[0,0] - rotation_matrix[1,1] - rotation_matrix[2,2]) * 2
                    w = (rotation_matrix[2,1] - rotation_matrix[1,2]) / S
                    x = 0.25 * S
                    y = (rotation_matrix[0,1] + rotation_matrix[1,0]) / S
                    z = (rotation_matrix[0,2] + rotation_matrix[2,0]) / S
                elif rotation_matrix[1,1] > rotation_matrix[2,2]:
                    S = np.sqrt(1.0 + rotation_matrix[1,1] - rotation_matrix[0,0] - rotation_matrix[2,2]) * 2
                    w = (rotation_matrix[0,2] - rotation_matrix[2,0]) / S
                    x = (rotation_matrix[0,1] + rotation_matrix[1,0]) / S
                    y = 0.25 * S
                    z = (rotation_matrix[1,2] + rotation_matrix[2,1]) / S
                else:
                    S = np.sqrt(1.0 + rotation_matrix[2,2] - rotation_matrix[0,0] - rotation_matrix[1,1]) * 2
                    w = (rotation_matrix[1,0] - rotation_matrix[0,1]) / S
                    x = (rotation_matrix[0,2] + rotation_matrix[2,0]) / S
                    y = (rotation_matrix[1,2] + rotation_matrix[2,1]) / S
                    z = 0.25 * S

            # Set orientation
            pose_msg.pose.orientation.w = w
            pose_msg.pose.orientation.x = x
            pose_msg.pose.orientation.y = y
            pose_msg.pose.orientation.z = z

            # Publish the pose
            self.head_pose_pub.publish(pose_msg)
            rospy.loginfo(f'Head pose published at {data.timestamp}')

if __name__ == '__main__':
    try:
        node = HoloLensSINode()
        node.start()
    except rospy.ROSInterruptException:
        pass
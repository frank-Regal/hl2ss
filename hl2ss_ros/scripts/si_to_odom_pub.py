#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_matrix, quaternion_matrix, euler_from_matrix
import numpy as np

from viewer import hl2ss
from viewer import hl2ss_lnm

class HoloLensSINode:
    def __init__(self):
        rospy.init_node('si_to_odom_node', anonymous=True)
        
        # ROS Parameters
        self.host = rospy.get_param('~host', '192.168.11.33')
        
        # Publishers
        self.odom_pub = rospy.Publisher('hololens/odom', Odometry, queue_size=10)
        
        self.client = None

    def start(self):
        # Initialize the spatial input client with the host address
        self.client = hl2ss_lnm.rx_si(self.host, hl2ss.StreamPort.SPATIAL_INPUT)
        self.client.open()
        
        # Initialize previous state variables for velocity calculations
        self.prev_position = None
        self.prev_orientation = None
        self.prev_time = None

        # Set up ROS rate limiter for 30 Hz updates
        rate = rospy.Rate(30)  # 30 Hz sample rate
        
        # Main processing loop
        while not rospy.is_shutdown():
            self.process_data()
            rate.sleep()

        # Clean up by closing the client connection
        self.client.close()
        

    def process_data(self):
        data = self.client.get_next_packet()
        si = hl2ss.unpack_si(data.payload)

        if si.is_valid_head_pose():
            head_pose = si.get_head_pose()
            
            # print(f'Head pose: Position={head_pose.position} Forward={head_pose.forward} Up={head_pose.up}')
            
            # Set position in ROS coordinates
            position_rh_ros_coords = np.array([
                -head_pose.position[2], 
                -head_pose.position[0], 
                head_pose.position[1]
                ])
            
            # Get orientation
            forward = -np.array(head_pose.forward) # -z axis
            up = np.array(head_pose.up)            # y axis
            right = np.cross(up, forward)          # x axis
            rot = np.array([right, up, forward]).T # rotation matrix
            rotmat = np.eye(4)
            rotmat[:3, :3] = rot
            quaternion = quaternion_from_matrix(rotmat) # create quaternion from rotation matrix
            
            # Set orientation in ROS coordinates
            quaternion_rh_ros_coords = np.array([
                 quaternion[2], # x
                 quaternion[0], # y
                -quaternion[1], # z
                -quaternion[3]  # w
                ])


            # Calculate velocities
            linear_vel, angular_vel = self.calcuate_twist(position_rh_ros_coords, quaternion_matrix(quaternion_rh_ros_coords))
                
            # Create Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "world"
            odom_msg.child_frame_id = "hololens"
            
            # Set position
            odom_msg.pose.pose.position.x = position_rh_ros_coords[0]
            odom_msg.pose.pose.position.y = position_rh_ros_coords[1] 
            odom_msg.pose.pose.position.z = position_rh_ros_coords[2]
            
            # Set orientation
            odom_msg.pose.pose.orientation.x = quaternion_rh_ros_coords[0]
            odom_msg.pose.pose.orientation.y = quaternion_rh_ros_coords[1]
            odom_msg.pose.pose.orientation.z = quaternion_rh_ros_coords[2]
            odom_msg.pose.pose.orientation.w = quaternion_rh_ros_coords[3]
            
            # Set linear velocity
            odom_msg.twist.twist.linear.x = linear_vel[0]
            odom_msg.twist.twist.linear.y = linear_vel[1] 
            odom_msg.twist.twist.linear.z = linear_vel[2]

            # Set angular velocity (currently not available from HoloLens)
            odom_msg.twist.twist.angular.x = angular_vel[0]
            odom_msg.twist.twist.angular.y = angular_vel[1] 
            odom_msg.twist.twist.angular.z = angular_vel[2]

            # Set covariance matrices
            odom_msg.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                                        0, 0.01, 0, 0, 0, 0,
                                        0, 0, 0.01, 0, 0, 0,
                                        0, 0, 0, 0.01, 0, 0,
                                        0, 0, 0, 0, 0.01, 0,
                                        0, 0, 0, 0, 0, 0.01]
            
            odom_msg.twist.covariance = [0.01, 0, 0, 0, 0, 0,
                                         0, 0.01, 0, 0, 0, 0, 
                                         0, 0, 0.01, 0, 0, 0,
                                         0, 0, 0, 0.01, 0, 0,
                                         0, 0, 0, 0, 0.01, 0,
                                         0, 0, 0, 0, 0, 0.01]

            # Publish odometry message
            self.odom_pub.publish(odom_msg)
            rospy.loginfo(f'Odometry published at {rospy.Time.now()}')
        
    def calcuate_twist(self, current_position, current_orientation):
        
        # Calculate velocities if we have previous pose
        if self.prev_position is not None and self.prev_time is not None:
            # Time difference
            dt = (rospy.Time.now() - self.prev_time).to_sec()
            
            if dt > 0:
                # Linear velocity (m/s)
                linear_vel = (current_position - self.prev_position) / dt
                
                # Angular velocity (rad/s)
                rel_rot = np.dot(current_orientation, self.prev_orientation.T)
                
                # Convert to euler angles (roll, pitch, yaw)
                angles = np.array(euler_from_matrix(rel_rot, 'sxyz'))
                angular_vel = angles / dt
        else:
            linear_vel = np.zeros(3)
            angular_vel = np.zeros(3)
        
        # Update previous pose and time
        self.prev_position = current_position
        self.prev_orientation = current_orientation
        self.prev_time = rospy.Time.now()
        
        return linear_vel, angular_vel
                
if __name__ == '__main__':
    try:
        node = HoloLensSINode()
        node.start()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import numpy as np
from tf.transformations import quaternion_from_matrix

from viewer import hl2ss
from viewer import hl2ss_lnm

class HoloLensSItoIMUNode:
    def __init__(self):
        rospy.init_node('hololens_si_imu_node', anonymous=True)
        
        # ROS Parameters
        self.host = rospy.get_param('~host', '192.168.11.33')
        
        # Publishers
        self.imu_pub = rospy.Publisher('hololens/imu', Imu, queue_size=10)
        
        self.client = None
        self.prev_forward = None
        self.prev_up = None
        self.prev_time = None

    def compute_angular_velocity(self, forward, up, current_time):
        """Compute angular velocity from consecutive orientations"""
        if self.prev_forward is None or self.prev_up is None or self.prev_time is None:
            self.prev_forward = forward
            self.prev_up = up
            self.prev_time = current_time
            return [0, 0, 0]

        # Calculate time difference
        dt = (current_time - self.prev_time).to_sec()
        if dt == 0:
            return [0, 0, 0]

        # Get rotation matrices for both orientations using ROS coordinate system
        forward = -forward  # -z axis
        right = np.cross(up, forward)  # x axis
        # up remains as y axis
        
        prev_forward = -self.prev_forward
        prev_right = np.cross(self.prev_up, prev_forward)
        
        current_matrix = np.array([right, up, forward]).T
        prev_matrix = np.array([prev_right, self.prev_up, prev_forward]).T

        # Ensure matrices are orthogonal
        current_matrix = np.array([
            current_matrix[:, 0] / np.linalg.norm(current_matrix[:, 0]),
            current_matrix[:, 1] / np.linalg.norm(current_matrix[:, 1]),
            current_matrix[:, 2] / np.linalg.norm(current_matrix[:, 2])
        ]).T
        
        prev_matrix = np.array([
            prev_matrix[:, 0] / np.linalg.norm(prev_matrix[:, 0]),
            prev_matrix[:, 1] / np.linalg.norm(prev_matrix[:, 1]),
            prev_matrix[:, 2] / np.linalg.norm(prev_matrix[:, 2])
        ]).T

        # Calculate rotation difference
        rotation_diff = np.dot(current_matrix, prev_matrix.T)
        
        # Convert to euler angles
        euler = np.array([
            np.arctan2(rotation_diff[2,1], rotation_diff[2,2]),
            np.arcsin(-rotation_diff[2,0]),
            np.arctan2(rotation_diff[1,0], rotation_diff[0,0])
        ])
        
        # Calculate angular velocity
        angular_vel = euler / dt
        
        # Update previous values
        self.prev_forward = forward
        self.prev_up = up
        self.prev_time = current_time
        
        return angular_vel

    def start(self):
        self.client = hl2ss_lnm.rx_si(self.host, hl2ss.StreamPort.SPATIAL_INPUT)
        self.client.open()

        rate = rospy.Rate(30)  # 30 Hz sample rate
        while not rospy.is_shutdown():
            self.process_data()
            rate.sleep()

        self.client.close()

    def print_data(self, si):
        print(f'Tracking status at time {rospy.Time.now()}')
        if (si.is_valid_head_pose()):
            head_pose = si.get_head_pose()
            print(f'Head pose: Position={head_pose.position} Forward={head_pose.forward} Up={head_pose.up}')
            # right = cross(up, -forward)
            # up => y, forward => -z, right => x
        else:
            print('No head pose data')
        if (si.is_valid_eye_ray()):
            eye_ray = si.get_eye_ray()
            print(f'Eye ray: Origin={eye_ray.origin} Direction={eye_ray.direction}')
        else:
            print('No eye tracking data')
        # See
        # https://learn.microsoft.com/en-us/uwp/api/windows.perception.people.jointpose?view=winrt-22621
        # for hand data details
        if (si.is_valid_hand_left()):
            hand_left = si.get_hand_left()
            pose = hand_left.get_joint_pose(hl2ss.SI_HandJointKind.Wrist)
            print(f'Left wrist pose: Position={pose.position} Orientation={pose.orientation} Radius={pose.radius} Accuracy={pose.accuracy}')
        else:
            print('No left hand data')
        if (si.is_valid_hand_right()):
            hand_right = si.get_hand_right()
            pose = hand_right.get_joint_pose(hl2ss.SI_HandJointKind.Wrist)
            print(f'Right wrist pose: Position={pose.position} Orientation={pose.orientation} Radius={pose.radius} Accuracy={pose.accuracy}')
        else:
            print('No right hand data')
            
    def process_data(self):
        data = self.client.get_next_packet()
        si = hl2ss.unpack_si(data.payload)
        
        if si.is_valid_head_pose():
            head_pose = si.get_head_pose()
            current_time = rospy.Time.now()
            
            # Create IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = current_time
            imu_msg.header.frame_id = "world"

            # Convert to right-handed ROS coordinate system
            forward = -np.array(head_pose.forward)  # -z axis
            up = np.array(head_pose.up)            # y axis
            right = np.cross(up, forward)          # x axis
            
            # Calculate angular velocity
            angular_vel = self.compute_angular_velocity(forward, up, current_time)
            
            # Set angular velocity (needs to be transformed to ROS coordinate system)
            imu_msg.angular_velocity.x = angular_vel[2]    # roll
            imu_msg.angular_velocity.y = angular_vel[0]    # pitch
            imu_msg.angular_velocity.z = -angular_vel[1]   # yaw

            # Create rotation matrix
            rotation_matrix = np.array([right, up, forward]).T
            rotmat = np.eye(4)
            rotmat[:3, :3] = rotation_matrix

            # Convert to quaternion [x, y, z, w]
            quaternion = quaternion_from_matrix(rotmat)

            # Set orientation with proper coordinate transformation
            imu_msg.orientation.w = -quaternion[3]
            imu_msg.orientation.x = quaternion[2]
            imu_msg.orientation.y = quaternion[0]
            imu_msg.orientation.z = -quaternion[1]

            # Set covariances to unknown
            imu_msg.orientation_covariance = [-1] * 9
            imu_msg.angular_velocity_covariance = [-1] * 9
            imu_msg.linear_acceleration_covariance = [-1] * 9

            # Publish IMU message
            self.imu_pub.publish(imu_msg)

if __name__ == '__main__':
    try:
        node = HoloLensSItoIMUNode()
        node.start()
    except rospy.ROSInterruptException:
        pass
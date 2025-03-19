#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from tf.transformations import quaternion_from_matrix, quaternion_matrix
import sys
import tty
import termios
import select
import os
import datetime
import pdb


"""
Class to listen to and store TF transforms between specified frames
"""
class TfListener:
    # Initialize with lists of source and target frames to track
    def __init__(self, frames=[]):
        # Create TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Store frame lists
        self.frames = frames

        # Initialize map to store transforms between each source-target pair
        self.tf_map = {self.formatted_key(source_frame, target_frame): None for source_frame, target_frame in frames}
        self.tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=10)

    # Helper to format dictionary keys for transform pairs
    def formatted_key(self, source_frame, target_frame):
        return f'{source_frame}_to_{target_frame}'

    # Look up transform between two frames
    def get_transform(self, source_frame, target_frame):
        try:
            # Get latest transform
            transform = self.tf_buffer.lookup_transform(source_frame, target_frame, rospy.Time(0))
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            clear()
            rospy.logwarn(f"{source_frame}_to_{target_frame} does not exist")
            clear()
            return None

    # Update all transforms in the map
    def update_transforms(self):
        for key in self.tf_map.keys():
            # Split key into source and target frames
            source_frame, target_frame = key.split('_to_')

            # Look up and store transform
            # republish transform to camera frame with april tag now in ros coordinate frames
            if key == 'rm_vlc_leftfront_to_april_tag':
                repub_tf = self.get_transform(source_frame, target_frame)
                if repub_tf is not None:
                    repub_tf = self.camera_to_ros_transform(repub_tf)
                    self.tf_pub.publish([repub_tf])
                    self.tf_map[key] = repub_tf
                else:
                    pass
            else:
                self.tf_map[key] = self.get_transform(source_frame, target_frame)

    # Print all transforms in the map
    def print_transforms(self):
        for key in self.tf_map.keys():
            print(f'{key}: {self.tf_map[key]}')

    # Write all transforms to a file
    def write_transforms(self):
        for key in self.tf_map.keys():
            self.LOG(f'{key}: {self.tf_map[key]}')

    # Log message to file
    def LOG(self, message):
        with open('log.txt', 'a') as f:
            f.write(f'{message}\n')

    # Convert camera convention to ROS convention (position)
    #  Camera convention is x=right,   y=-up,    z=forward
    #  ROS convention is    x=forward, y=-right, z=up
    def camera_to_ros_position(self, tf_camera_convention):
        return np.array([tf_camera_convention.transform.translation.z, -tf_camera_convention.transform.translation.x, -tf_camera_convention.transform.translation.y])

    # Convert camera convention to ROS convention (rotation)
    #  Camera convention is x=right,   y=-up,    z=forward
    #  ROS convention is    x=forward, y=-right, z=up
    def camera_to_ros_rotation(self, tf_camera_convention):
        return np.array([tf_camera_convention.transform.rotation.z,-tf_camera_convention.transform.rotation.x, -tf_camera_convention.transform.rotation.y, tf_camera_convention.transform.rotation.w])

    # Convert camera convention to ROS convention (transform)
    #  Camera convention is x=right,   y=-up,    z=forward
    #  ROS convention is    x=forward, y=-right, z=up
    def camera_to_ros_transform(self, tf_camera_convention):
        # Create transform stamped message
        tf_camera = TransformStamped()
        tf_camera.header.stamp = rospy.Time.now()
        tf_camera.header.frame_id = 'camera'
        tf_camera.child_frame_id = 'april'

        # Get position and rotation in ROS convention
        pos = self.camera_to_ros_position(tf_camera_convention)
        rot = self.camera_to_ros_rotation(tf_camera_convention)

        # Convert quaternion to rotation matrix
        rot_mat_so3 = quaternion_matrix(rot)
        rot_mat_so3 = np.array([rot_mat_so3[0][0:3], rot_mat_so3[1][0:3], rot_mat_so3[2][0:3]])

        # Apply rotations to align coordinate frames
        rot_mat_so3 = self.rotate_about_axis(rot_mat_so3, -90, 'x', 'body')
        rot_mat_so3 = self.rotate_about_axis(rot_mat_so3, 180, 'z', 'body')

        # Convert back to quaternion
        rot_mat_se3 = np.eye(4)
        rot_mat_se3[0:3, 0:3] = rot_mat_so3
        rot_manip = quaternion_from_matrix(rot_mat_se3)

        # Set transform values
        tf_camera.transform.translation.x = pos[0]
        tf_camera.transform.translation.y = pos[1]
        tf_camera.transform.translation.z = pos[2]
        tf_camera.transform.rotation.x = rot_manip[0]
        tf_camera.transform.rotation.y = rot_manip[1]
        tf_camera.transform.rotation.z = rot_manip[2]
        tf_camera.transform.rotation.w = rot_manip[3]

        return tf_camera

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

    # Get position of target frame relative to source frame
    def get_position(self, source_frame, target_frame):
        tf = self.tf_map[self.formatted_key(source_frame, target_frame)]
        if tf is not None:
            return np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
        else:
            return None

    # Get rotation of target frame relative to source frame
    def get_rotation(self, source_frame, target_frame):
        tf = self.tf_map[self.formatted_key(source_frame, target_frame)]
        if tf is not None:
            return np.array([tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w])
        else:
            return None

class Logger:
    def __init__(self, base_path, file_name):
        self.timestamp = None
        self.file_path = None
        self.file_name = file_name
        self.base_path = base_path

    def format_data(self, t_Robot_config, q_Robot_config, t_camera_config, q_camera_config):
        return f'{float(t_Robot_config[0]):.4f}, {float(t_Robot_config[1]):.4f}, {float(t_Robot_config[2]):.4f}; {float(q_Robot_config[0]):.4f}, {float(q_Robot_config[1]):.4f}, {float(q_Robot_config[2]):.4f}, {float(q_Robot_config[3]):.4f}; {float(t_camera_config[0]):.4f}, {float(t_camera_config[1]):.4f}, {float(t_camera_config[2]):.4f}; {float(q_camera_config[0]):.4f}, {float(q_camera_config[1]):.4f}, {float(q_camera_config[2]):.4f}, {float(q_camera_config[3]):.4f};'

    def write_to_file(self, data):
        with open(self.file_path, 'a') as f:
            f.write(f'{data}\n')

    def setup_file(self):
        if self.file_path is None:
            os.makedirs(os.path.join(self.base_path), exist_ok=True)
            self.file_name = self.file_name + '.txt'
            self.file_path = os.path.join(self.base_path, self.file_name)
            with open(self.file_path, 'a') as f:
                f.write('t_Robot_config_x, t_Robot_config_y, t_Robot_config_z; q_Robot_config_x, q_Robot_config_y, q_Robot_config_z, q_Robot_config_w; t_camera_config_x, t_camera_config_y, t_camera_config_z; q_camera_config_x, q_camera_config_y, q_camera_config_z, q_camera_config_w\n')

def is_data():
    """Check if there is data waiting on stdin"""
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def getch():
    """Gets a single character from stdin without blocking"""
    if is_data():
        return sys.stdin.read(1)
    return None

def clear():
    sys.stdout.write('\r')
    sys.stdout.flush()
    print(f'', end="\r")

def collect_data(tf_listener, logger):

    # Create file for this trial

    logger.setup_file()

    # Get positions
    t_Robot_config = tf_listener.get_position(source_frame='world', target_frame='rignode')
    q_Robot_config = tf_listener.get_rotation(source_frame='world', target_frame='rignode')
    t_camera_config = tf_listener.get_position(source_frame='camera', target_frame='april')
    q_camera_config = tf_listener.get_rotation(source_frame='camera', target_frame='april')

    clear()
    if t_Robot_config is not None and t_camera_config is not None and q_Robot_config is not None and q_camera_config is not None:

        # Format data for writing to file
        data = logger.format_data(t_Robot_config=t_Robot_config,
                                  q_Robot_config=q_Robot_config,
                                  t_camera_config=t_camera_config,
                                  q_camera_config=q_camera_config)

        # Write data to file
        logger.write_to_file(data)

        # Log completion of trial
        rospy.loginfo(f"Data written.")
        clear()

        return

    else:
        print(f"Failed to collect data")
        clear()
        return None

def main():
    # Initialize ROS node
    rospy.init_node('calibration_user_input', anonymous=True)
    filename = rospy.get_param('~filename', 'calibration_data')

    # Set base path for data
    base_path = '/home/frank/project/ws_dev/src/hl2ss/hl2ss_ros/config' # no trailing slash

    # Tf Listener expects a list of tuples, where each tuple contains two strings
    # representing the source frame first and target frame second e.g. [(source_frame, target_frame), ...]
    tf_listener = TfListener(frames=[('world', 'rignode'),
                                     ('rm_vlc_leftfront', 'april_tag'),
                                     ('camera', 'april')])
    # Logger to write data to file
    logger = Logger(base_path=base_path, file_name=filename)

    # Set up terminal for non-blocking input
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)


    # Print instructions
    print('Ready to record...')
    print('')

    print("Press '1' to record data")
    print("Press 'q' to quit")

    # Wait for user input
    try:
        tty.setraw(fd)
        while not rospy.is_shutdown():
            tf_listener.update_transforms()

            char = getch()
            if char is not None:

                # Exit program
                if char == 'q':
                    sys.stdout.write('\r')
                    sys.stdout.flush()
                    print(f"Exiting...", end="\r")
                    print(f'\n')
                    break

                # Write Data to File
                elif char == '1':
                    collect_data(tf_listener=tf_listener, logger=logger)

            rospy.sleep(0.2)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np

import sys
import tty
import termios
import select
import os
import datetime


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

    # Get positions of the QR code, yellow, and pink
    t_Robot_config = tf_listener.get_position(source_frame='world', target_frame='rignode')
    q_Robot_config = tf_listener.get_rotation(source_frame='world', target_frame='rignode')
    t_camera_config = tf_listener.get_position(source_frame='vlc', target_frame='april_tag')
    q_camera_config = tf_listener.get_rotation(source_frame='vlc', target_frame='april_tag')

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

    # Set base path for data
    base_path = '/home/frank/project/ws_dev/src/hl2ss/hl2ss_ros/config' # no trailing slash

    # Tf Listener expects a list of tuples, where each tuple contains two strings
    # representing the source frame first and target frame second e.g. [(source_frame, target_frame), ...]
    tf_listener = TfListener(frames=[('world', 'rignode'),
                                     ('vlc', 'april_tag')])

    # Logger to write data to file
    logger = Logger(base_path=base_path, file_name='calibration_data')

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

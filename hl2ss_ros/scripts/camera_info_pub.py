#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CameraInfo
from camera_info_manager import CameraInfoManager

def main():
    rospy.init_node('camera_info_publisher')

    ag_n = rospy.get_param('~ag_n','0')
    camera_info_path = rospy.get_param('~camera_info_path','')

    # Create a publisher for the camera_info topic
    info_pub = rospy.Publisher('camera_info', CameraInfo, queue_size=10)

    # Initialize the CameraInfoManager
    camera_name = f'hololens_ag{ag_n}'
    camera_info_manager = CameraInfoManager(cname=camera_name, url=camera_info_path)

    # Load the calibration data
    camera_info_manager.loadCameraInfo()

    # Get the CameraInfo message
    camera_info = camera_info_manager.getCameraInfo()

    rate = rospy.Rate(10)  # 10 Hz

    rospy.loginfo(f"Publishing camera info for '{camera_name}'...")

    while not rospy.is_shutdown():
        # Update the timestamp
        camera_info.header.stamp = rospy.Time.now()

        # Publish the camera info
        info_pub.publish(camera_info)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
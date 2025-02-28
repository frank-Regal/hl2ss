#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
import tf2_ros
import math


class StaticTransformsPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('static_transforms_publisher', anonymous=True)

        # Create a StaticTransformBroadcaster
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Publish the static transforms
        self.publish_static_transforms()

    def publish_static_transforms(self):
        # First static transform
        hololens_to_rignode = TransformStamped()
        hololens_to_rignode.header.stamp = rospy.Time.now()
        hololens_to_rignode.header.frame_id = 'hololens_ag0/base_link'
        hololens_to_rignode.child_frame_id = 'rignode'
        hololens_to_rignode.transform.translation.x = 0.0525
        hololens_to_rignode.transform.translation.y = 0.0481
        hololens_to_rignode.transform.translation.z = 0.0422

        # Set rotation using roll, pitch, yaw
        quaternion = quaternion_from_euler(math.radians(-1.2180), math.radians(5.8608), math.radians(0.3203), axes='sxyz')
        hololens_to_rignode.transform.rotation.x = quaternion[0]
        hololens_to_rignode.transform.rotation.y = quaternion[1]
        hololens_to_rignode.transform.rotation.z = quaternion[2]
        hololens_to_rignode.transform.rotation.w = quaternion[3]

        # Second static transform
        hololens_to_rightfront = TransformStamped()
        hololens_to_rightfront.header.stamp = rospy.Time.now()
        hololens_to_rightfront.header.frame_id = 'hololens_ag0/base_link'
        hololens_to_rightfront.child_frame_id = 'rightfront'
        hololens_to_rightfront.transform.translation.x = 0.0525
        hololens_to_rightfront.transform.translation.y = -0.0481
        hololens_to_rightfront.transform.translation.z = 0.0422

        # Set rotation using roll, pitch, yaw
        quaternion = quaternion_from_euler(math.radians(1.2180), math.radians(5.8608), math.radians(-0.3203), axes='sxyz')
        hololens_to_rightfront.transform.rotation.x = quaternion[0]
        hololens_to_rightfront.transform.rotation.y = quaternion[1]
        hololens_to_rightfront.transform.rotation.z = quaternion[2]
        hololens_to_rightfront.transform.rotation.w = quaternion[3]
        
        # Second static transform
        rignode_to_rightfront = TransformStamped()
        rignode_to_rightfront.header.stamp = rospy.Time.now()
        rignode_to_rightfront.header.frame_id = 'rignode'
        rignode_to_rightfront.child_frame_id = 'rm_vlc_rightfront'
        rignode_to_rightfront.transform.translation.x = -0.001
        rignode_to_rightfront.transform.translation.y = -0.096
        rignode_to_rightfront.transform.translation.z = -0.002

        # Set rotation using roll, pitch, yaw
        rignode_to_rightfront.transform.rotation.x = 0.022
        rignode_to_rightfront.transform.rotation.y = 0.000
        rignode_to_rightfront.transform.rotation.z = -0.006
        rignode_to_rightfront.transform.rotation.w = 1.000
        

        # Broadcast the transforms
        self.static_broadcaster.sendTransform([hololens_to_rignode, rignode_to_rightfront])

if __name__ == '__main__':
    try:
        StaticTransformsPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

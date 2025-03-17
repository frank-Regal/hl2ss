#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
import tf2_ros
import math

# -----------------------------------------------------------------------------
# HoloLens /base_link (Unity Headset Frame) to Rignode Transform (hl2ss.data)
# -----------------------------------------------------------------------------
class BaselinkToRignode:
    BASEFRAME =  rospy.get_param('hololens_baselink','hololens_ag0/base_link')
    CHILDFRAME = rospy.get_param('hololens_rignode','rignode')
    X = 0.0525
    Y = 0.0481
    Z = 0.0422
    QUAT = quaternion_from_euler(math.radians(-1.2180),
                                 math.radians(5.8608),
                                 math.radians(0.3203),
                                 axes='sxyz')
# -----------------------------------------------------------------------------
# Rignode to Left Front Camera Transform
# -----------------------------------------------------------------------------
class RignodeToLeftFront:
    BASEFRAME =  rospy.get_param('hololens_rignode','rignode')
    CHILDFRAME = rospy.get_param('hololens_rm_vlc_leftfront','rm_vlc_leftfront')
    X = 0
    Y = 0
    Z = 0
    QUAT = [0, 0, 0, 1]

# -----------------------------------------------------------------------------
# Rignode to Right Front Camera Transform (Obtained from Kalibr)
# -----------------------------------------------------------------------------
class RignodeToRightFront:
    BASEFRAME =  rospy.get_param('hololens_rignode','rignode')
    CHILDFRAME = rospy.get_param('hololens_rm_vlc_rightfront','rm_vlc_rightfront')
    X =  0.0006
    Y = -0.0990
    Z = -0.0013
    QUAT = [0.0120, 0.0065, 0.0031, 0.9999]

# -----------------------------------------------------------------------------
# Hololens Static TF Publisher
# -----------------------------------------------------------------------------
class HololensStaticTfPub:
    '''
    Initialize the Hololens Static TF Publisher
    '''
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('hololens_static_tf_pub', anonymous=True)

        # Create a StaticTransformBroadcaster
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Create a list of static transforms
        tf_list = [
            BaselinkToRignode,
            RignodeToLeftFront,
            RignodeToRightFront
        ]

        # Publish the static transforms
        self.publish_static_tf(tf_list)

    '''
    Publish Static Transforms
    '''
    def publish_static_tf(self, tf_list):
        tf_msg_list = []

        # Build the static transforms
        for tf in tf_list:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = rospy.Time.now()
            tf_msg.header.frame_id = tf.BASEFRAME
            tf_msg.child_frame_id = tf.CHILDFRAME
            tf_msg.transform.translation.x = tf.X
            tf_msg.transform.translation.y = tf.Y
            tf_msg.transform.translation.z = tf.Z
            tf_msg.transform.rotation.x = tf.QUAT[0]
            tf_msg.transform.rotation.y = tf.QUAT[1]
            tf_msg.transform.rotation.z = tf.QUAT[2]
            tf_msg.transform.rotation.w = tf.QUAT[3]
            tf_msg_list.append(tf_msg)

        # Publish the static transforms
        self.static_broadcaster.sendTransform(tf_msg_list)

if __name__ == '__main__':
    try:
        HololensStaticTfPub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

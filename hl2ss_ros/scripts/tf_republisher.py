#!/usr/bin/env python3

import rospy
from tf2_msgs.msg import TFMessage

class TFRepublisher:
    def __init__(self):
        rospy.init_node('tf_republisher', anonymous=True)

        # Publisher for /tf topic
        self.tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=10)

        # Subscriber to /augre/tf topic
        rospy.Subscriber('/augre/tf', TFMessage, self.tf_callback)

        rospy.loginfo("TF Republisher node started. Republishing /augre/tf to /tf")

    def tf_callback(self, msg):
        # Simply republish the received TF message to the /tf topic
        rospy.loginfo(f"Received TF message with {len(msg.transforms)} transforms")
        self.tf_pub.publish(msg)

def main():
    try:
        republisher = TFRepublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

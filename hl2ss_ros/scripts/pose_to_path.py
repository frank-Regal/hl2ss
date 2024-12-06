#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
from tf.transformations import quaternion_matrix

class PoseToPathNode:
    def __init__(self):
        rospy.init_node('pose_to_path_node', anonymous=True)
        
        # Parameters
        self.max_poses = rospy.get_param('~max_poses', 1000)
        self.frame_id = "world"
        
        # Initialize messages
        self.path = Path()
        self.path.header.frame_id = self.frame_id
        
        self.pose_array = PoseArray()
        self.pose_array.header.frame_id = self.frame_id
        
        self.markers = MarkerArray()
        
        # Publishers
        self.path_pub = rospy.Publisher('hololens/head_path', Path, queue_size=10)
        self.pose_array_pub = rospy.Publisher('hololens/head_poses', PoseArray, queue_size=10)
        self.marker_pub = rospy.Publisher('hololens/head_markers', MarkerArray, queue_size=10)
        
        # Subscriber
        self.pose_sub = rospy.Subscriber('/hololens_ag0/3802/vlc_pose', PoseStamped, self.pose_callback)
        
        # Initialize visualization markers
        self.setup_markers()

    def setup_markers(self):
        # Marker for current head position (sphere)
        self.head_marker = Marker()
        self.head_marker.header.frame_id = self.frame_id
        self.head_marker.type = Marker.SPHERE
        self.head_marker.action = Marker.ADD
        self.head_marker.scale.x = 0.1
        self.head_marker.scale.y = 0.1
        self.head_marker.scale.z = 0.1
        self.head_marker.color.a = 1.0
        self.head_marker.color.r = 1.0
        self.head_marker.color.g = 0.0
        self.head_marker.color.b = 0.0
        self.head_marker.id = 0

        # Marker for gaze direction (arrow)
        self.gaze_marker = Marker()
        self.gaze_marker.header.frame_id = self.frame_id
        self.gaze_marker.type = Marker.ARROW
        self.gaze_marker.action = Marker.ADD
        self.gaze_marker.scale.x = 0.05  # shaft diameter
        self.gaze_marker.scale.y = 0.1   # head diameter
        self.gaze_marker.scale.z = 0.1   # head length
        self.gaze_marker.color.a = 1.0
        self.gaze_marker.color.r = 0.0
        self.gaze_marker.color.g = 1.0
        self.gaze_marker.color.b = 0.0
        self.gaze_marker.id = 1

        # Marker for path (line strip with gradient color)
        self.path_marker = Marker()
        self.path_marker.header.frame_id = self.frame_id
        self.path_marker.type = Marker.LINE_STRIP
        self.path_marker.action = Marker.ADD
        self.path_marker.scale.x = 0.02  # line width
        self.path_marker.pose.orientation.w = 1.0
        self.path_marker.id = 2

    def update_markers(self, pose_msg):
        now = rospy.Time.now()
        
        # Update head position marker
        self.head_marker.header.stamp = now
        self.head_marker.pose = pose_msg.pose

        # Update gaze direction marker
        self.gaze_marker.header.stamp = now
        self.gaze_marker.points = []
        self.gaze_marker.points.append(pose_msg.pose.position)
        
        # Calculate forward point using pose orientation
        quat = pose_msg.pose.orientation
        rot_matrix = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
        forward_local = np.array([0, 0, 1, 1])  # Forward in local frame
        forward_world = np.dot(rot_matrix, forward_local)
        
        end_point = pose_msg.pose.position
        end_point.x += forward_world[0] * 0.3  # 30cm forward
        end_point.y += forward_world[1] * 0.3
        end_point.z += forward_world[2] * 0.3
        self.gaze_marker.points.append(end_point)

        # Update path marker
        self.path_marker.header.stamp = now
        self.path_marker.points.append(pose_msg.pose.position)
        if len(self.path_marker.points) > self.max_poses:
            self.path_marker.points.pop(0)

        # Create color gradient for path
        self.path_marker.colors = []
        n_points = len(self.path_marker.points)
        for i in range(n_points):
            color = ColorRGBA()
            color.a = 1.0
            # Gradient from blue (old) to red (new)
            color.r = i / float(n_points)
            color.b = 1.0 - (i / float(n_points))
            color.g = 0.0
            self.path_marker.colors.append(color)

    def pose_callback(self, pose_msg):
        # Update path
        self.path.poses.append(pose_msg)
        if len(self.path.poses) > self.max_poses:
            self.path.poses.pop(0)
        self.path.header.stamp = rospy.Time.now()
        
        # Update pose array
        self.pose_array.poses.append(pose_msg.pose)
        if len(self.pose_array.poses) > self.max_poses:
            self.pose_array.poses.pop(0)
        self.pose_array.header.stamp = rospy.Time.now()
        
        # Update markers
        self.update_markers(pose_msg)
        
        # Publish everything
        self.path_pub.publish(self.path)
        self.pose_array_pub.publish(self.pose_array)
        
        # Publish markers
        marker_array = MarkerArray()
        marker_array.markers = [self.head_marker, self.gaze_marker, self.path_marker]
        self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        node = PoseToPathNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
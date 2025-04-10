#!/usr/bin/env python3

import rospy
import tf.transformations
import numpy as np

from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, \
    InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState


if __name__ == "__main__":
    rospy.init_node("test_trajectory")

    # TODO: Pre-compute intermediate poses of trajectory @Ryan

    pose_pub = rospy.Publisher(
        "equilibrium_pose", PoseStamped, queue_size=10)

    rate = rospy.Rate(1) # Hz (TODO: probably want faster)
    pose = PoseStamped()

    ARRAY_SIZE = 100
    RADIUS = 0.25
    OFFSET_X = 0.4
    OFFSET_Y = 0.0

    theta = np.linspace(0, np.pi, ARRAY_SIZE)

    x = OFFSET_X
    y = RADIUS * np.cos(theta)
    z = RADIUS * np.sin(theta)

    pose.pose.position.x = x[0]
    pose.pose.position.y = y[0]
    pose.pose.position.z = z[0]
    
    # Rotation as quaternion (no rotation in this example)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0


    i = 0
    while not rospy.is_shutdown():
        pose.header.stamp = rospy.Time.now()
        # pose.header.frame_id = "base_link"

        pose.pose.position.x = x[i]
        pose.pose.position.y = y[i]
        pose.pose.position.z = z[i]
        pose_pub.publish(pose)
        i += 1
        if i >= ARRAY_SIZE:
            rospy.signal_shutdown()
            
        rospy.loginfo("Test trajectory is running") # Just a test msg to see that script is running
        rate.sleep()
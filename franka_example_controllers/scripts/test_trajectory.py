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

    # pose_pub = rospy.Publisher(
    #     "equilibrium_pose", PoseStamped, queue_size=10)

    rate = rospy.Rate(1) # Hz (TODO: probably want faster)
    pose = PoseStamped()

    while not rospy.is_shutdown():
        # pose.header.stamp = rospy.Time.now()
        # pose_pub.publish(pose)
        rospy.loginfo("Test trajectory is running") # Just a test msg to see that script is running
        rate.sleep()
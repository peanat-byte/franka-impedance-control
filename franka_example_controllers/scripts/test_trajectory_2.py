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

import numpy as np
from scipy.spatial.transform import Rotation as R

def compute_quaternion_to_point_surface(center, surface_point):
    z_axis = np.array([0, 0, 1])
    normal = np.array(center) - np.array(surface_point)
    normal /= np.linalg.norm(normal)

    # Compute rotation between z_axis and normal
    rotation, _ = R.align_vectors([normal], [z_axis])  # align z-axis to normal
    quat = rotation.as_quat()  # returns [x, y, z, w]
    return quat

if __name__ == "__main__":
    rospy.init_node("test_trajectory")

    # TODO: Pre-compute intermediate poses of trajectory @Ryan

    pose_pub = rospy.Publisher(
        "impedance_controller_mech_464/equilibrium_pose", PoseStamped, queue_size=10)

    rate = rospy.Rate(32) # Hz (TODO: probably want faster)
    pose = PoseStamped()

    ARRAY_SIZE = 400
    RADIUS = 0.25
    OFFSET_X = 0.6
    OFFSET_Y = 0.0
    OFFSET_Z = 0.25

    phi = np.linspace(0, 2*np.pi, ARRAY_SIZE)
    theta = np.pi/4

    x = OFFSET_X
    y = 0.3 * np.sin(phi)
    z = OFFSET_Z

    CENTER = [0.6, 0, 0]

    pose.pose.position.x = x
    pose.pose.position.y = y[0]
    pose.pose.position.z = z
    
    # quat = compute_quaternion_to_point_surface([x, y, z])
    # Rotation as quaternion (no rotation in this example)
    pose.pose.orientation.x = 1.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.0

    i = 0
    while not rospy.is_shutdown():
        pose.header.stamp = rospy.Time.now()
        # pose.header.frame_id = "base_link"

        pose.pose.position.x = x
        pose.pose.position.y = y[i]
        pose.pose.position.z = z

        surface_point = [x, y[i], z]
        quat = compute_quaternion_to_point_surface(CENTER, surface_point)

        # pose.pose.orientation.x = quat[0]
        # pose.pose.orientation.y = quat[1]
        # pose.pose.orientation.z = quat[2]
        # pose.pose.orientation.w = quat[3]

        pose_pub.publish(pose)

        i += 1
        if i >= ARRAY_SIZE:
            i = 0
            # rospy.signal_shutdown()
            
        rospy.loginfo("Test trajectory is running") # Just a test msg to see that script is running
        rate.sleep()
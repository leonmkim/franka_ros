#!/usr/bin/env python

import rospy
import tf.transformations
import numpy as np

from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState

from sensor_msgs.msg import Joy
from math import pi

marker_pose = PoseStamped()
initial_pose_found = False
pose_pub = None
# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]


def publisherCallback(msg, link_name):
    marker_pose.header.frame_id = link_name
    marker_pose.header.stamp = rospy.Time(0)
    pose_pub.publish(marker_pose)


def franka_state_callback(msg):
    initial_quaternion = \
        tf.transformations.quaternion_from_matrix(
            np.transpose(np.reshape(msg.O_T_EE,
                                    (4, 4))))
    initial_quaternion = initial_quaternion / np.linalg.norm(initial_quaternion)
    marker_pose.pose.orientation.x = initial_quaternion[0]
    marker_pose.pose.orientation.y = initial_quaternion[1]
    marker_pose.pose.orientation.z = initial_quaternion[2]
    marker_pose.pose.orientation.w = initial_quaternion[3]
    marker_pose.pose.position.x = msg.O_T_EE[12]
    marker_pose.pose.position.y = msg.O_T_EE[13]
    marker_pose.pose.position.z = msg.O_T_EE[14]
    global initial_pose_found
    initial_pose_found = True


def joy_callback(msg):
    marker_pose.pose.position.x = max(min(marker_pose.pose.position.x - msg.axes[0], position_limits[0][1]), position_limits[0][0])
    marker_pose.pose.position.y = max(min(marker_pose.pose.position.y + msg.axes[1], position_limits[1][1]), position_limits[1][0])
    marker_pose.pose.position.z = max(min(marker_pose.pose.position.z - ((msg.axes[5] - 1) + (msg.axes[4] - 1)) / (2 * position_limits[2][1]) , position_limits[2][1]), position_limits[2][0])
    marker_pose.pose.orientation.x += msg.buttons[3] / pi
    marker_pose.pose.orientation.y -= msg.buttons[7] / pi
    marker_pose.pose.orientation.z += (msg.buttons[4] - msg.buttons[5]) / pi


if __name__ == "__main__":
    rospy.init_node("joy_pose_node")
    pose_pub = rospy.Publisher(
        "equilibrium_pose", PoseStamped, queue_size=10)
    state_sub = rospy.Subscriber("franka_state_controller/franka_states",
                                 FrankaState, franka_state_callback)
    joy_sub = rospy.Subscriber("joy", Joy, joy_callback)                             
    listener = tf.TransformListener()
    link_name = rospy.get_param("~link_name")

    
    # run pose publisher
    rospy.Timer(rospy.Duration(0.005),
                lambda msg: publisherCallback(msg, link_name))

  


    rospy.spin()

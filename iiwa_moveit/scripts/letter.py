#!/usr/bin/env python3

import rospy
import moveit_commander
import threading
import copy
import sys
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray

rospy.init_node('moveit_cartesian_path_example', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
# robot = moveit_commander.RobotCommander()
# scene = moveit_commander.PlanningSceneInterface()
# group = moveit_commander.MoveGroupCommander("manipulator")
robot = moveit_commander.RobotCommander(
        robot_description="/iiwa/robot_description",
        ns="/iiwa"
    )
scene = moveit_commander.PlanningSceneInterface(ns="/iiwa")
group = moveit_commander.MoveGroupCommander(
        "manipulator",
        robot_description="/iiwa/robot_description",
        ns="/iiwa"
    )


# Define the initial pose
initial_pose = group.get_current_pose().pose
print(initial_pose)
#make changes in waypoints only
# Define the waypoints for the "B" letter
# group_name = "manipulator"
# move_group = moveit_commander.MoveGroupCommander(group_name)

# joint_goal = group.get_current_joint_values()
# joint_goal[0] = 0
# joint_goal[1] = -tau / 8
# joint_goal[2] = 0
# joint_goal[3] = -tau / 4
# joint_goal[4] = 0
# joint_goal[5] = 0 # 1/6 of a turn
# joint_goal[6] = (np.pi) / 2

# group.go(joint_goal, wait=True)

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
# move_group.go(joint_goal, wait=True)

# # Calling ``stop()`` ensures that there is no residual movement
# group.stop()
# joint_goal = move_group.get_current_joint_values()
waypoints = []

# Define the Cartesian path for the "B" letter
#keep z value fixed, change x and y within 0.2

initial_pose.position.x = 0.0
initial_pose.position.y = 0.0
initial_pose.position.z = 0.8
waypoints.append(copy.deepcopy(initial_pose))

initial_pose.position.x = 0.0
initial_pose.position.y = 0.2
initial_pose.position.z = 0.8
waypoints.append(copy.deepcopy(initial_pose))

initial_pose.position.x = 0.1
initial_pose.position.y = 0.2
initial_pose.position.z = 1.0
waypoints.append(copy.deepcopy(initial_pose))


# initial_pose.position.x = 0.2
# initial_pose.position.y = 0.2
# initial_pose.position.z = 1.0
# waypoints.append(copy.deepcopy(initial_pose))

# initial_pose.position.x = 0.2
# initial_pose.position.y = 0.2
# initial_pose.position.z = 1.0
# waypoints.append(copy.deepcopy(initial_pose))

# initial_pose.position.x = 0.0
# initial_pose.position.y = 0.2
# initial_pose.position.z = 1.0
# waypoints.append(copy.deepcopy(initial_pose))



# initial_pose.position.x = 0.1
# initial_pose.position.y = 0.0
# initial_pose.position.z = 1.0
# waypoints.append(copy.deepcopy(initial_pose))

# initial_pose.position.x = 0.2
# initial_pose.position.y = 0.2
# initial_pose.position.z = 1.0
# waypoints.append(copy.deepcopy(initial_pose))









# initial_pose.position.x = 0.2
# initial_pose.position.y = 0.3
# initial_pose.position.z = 1.0
# waypoints.append(copy.deepcopy(initial_pose))

# initial_pose.position.x = 0.2
# initial_pose.position.y = 0.3
# initial_pose.position.z = 0.9
# waypoints.append(copy.deepcopy(initial_pose))


# initial_pose.position.x = 0.2
# initial_pose.position.y = 0.2
# initial_pose.position.z = 0.9
# waypoints.append(copy.deepcopy(initial_pose))


# initial_pose.position.x = 1.0
# initial_pose.position.y = 1.0
# initial_pose.position.z = 0.0
# waypoints.append(copy.deepcopy(initial_pose))

# initial_pose.position.x = 0.0
# initial_pose.position.y = 0.0
# initial_pose.position.z = 1
# waypoints.append(copy.deepcopy(initial_pose))

# initial_pose.position.x = 0.15
# initial_pose.position.y = -0.15
# initial_pose.position.z = 1
# waypoints.append(copy.deepcopy(initial_pose))

def visualize_array(waypoints, r, g, b):
    marker_array = MarkerArray()
    frame_id = group.get_planning_frame()  # Use the robot's planning frame
    for i, point in enumerate(waypoints):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point.position.x
        marker.pose.position.y = point.position.y
        marker.pose.position.z = point.position.z
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker_array.markers.append(marker)
    return marker_array

# Marker publishing thread
def marker_publish_thread():
    marker_pub = rospy.Publisher('trajectory_markers', MarkerArray, queue_size=1, latch=True)
    rate = rospy.Rate(10)  # Adjust the rate as needed

    while not rospy.is_shutdown():  # Check if ROS node is still running
        try:
            marker_array = visualize_array(waypoints, 1.0, 1.0, 0.0)
            marker_pub.publish(marker_array)

            # Sleep
            rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            # Handle ROS shutdown request gracefully
            pass
# Create and start the marker publishing thread
marker_thread = threading.Thread(target=marker_publish_thread)
marker_thread.daemon = True  # Allow the thread to exit when the main program exits
marker_thread.start()

# Plan and execute the Cartesian path
(plan, fraction) = group.compute_cartesian_path(
    waypoints,
    0.001,  # eef_step
    False   # jump_threshold
)

if fraction == 1.0:
    group.execute(plan, wait=True)
else:
    print("Path planning failed")

# Clean up
moveit_commander.roscpp_shutdown()


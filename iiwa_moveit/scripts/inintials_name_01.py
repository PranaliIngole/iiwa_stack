#!/usr/bin/env python3

import rospy
import moveit_commander
import threading
import copy
import sys
from visualization_msgs.msg import Marker, MarkerArray

rospy.init_node('moveit_cartesian_path_example', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

# Define the initial pose
initial_pose = group.get_current_pose().pose
print(initial_pose)

# Define the waypoints for the "P" letter
waypoints = []

# Define the Cartesian path for the "P" letter
initial_pose.position.x = 0.0
initial_pose.position.y = 0.0
initial_pose.position.z = 1
waypoints.append(copy.deepcopy(initial_pose))

initial_pose.position.x = 0
initial_pose.position.y = 0.1
initial_pose.position.z = 1
waypoints.append(copy.deepcopy(initial_pose))


initial_pose.position.x = 0.1
initial_pose.position.y = 0.1
initial_pose.position.z = 1
waypoints.append(copy.deepcopy(initial_pose))


initial_pose.position.x = 0.1
initial_pose.position.y = 0.05
initial_pose.position.z = 1
waypoints.append(copy.deepcopy(initial_pose))

initial_pose.position.x = 0
initial_pose.position.y = 0.05
initial_pose.position.z = 1
waypoints.append(copy.deepcopy(initial_pose))

initial_pose.position.x = 0.0
initial_pose.position.y = 0.0
initial_pose.position.z = 1
waypoints.append(copy.deepcopy(initial_pose))

# Function to visualize the waypoints
def visualize_array(waypoints, r, g, b):
    marker_array = MarkerArray()
    for i, point in enumerate(waypoints):
        marker = Marker()
        marker.header.frame_id = "world"
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
        if rospy.is_shutdown():
            break  # Exit the loop if ROS is shutting down

        marker_array = visualize_array(waypoints, 1.0, 1.0, 0.0)
        marker_pub.publish(marker_array)
        rate.sleep()
# Create and start the marker publishing thread
marker_thread = threading.Thread(target=marker_publish_thread)
marker_thread.daemon = True  # Allow the thread to exit when the main program exits
marker_thread.start()

# Plan and execute the Cartesian path
(plan, fraction) = group.compute_cartesian_path(
    waypoints,
    0.001,  # eef_step
    0.0,   # jump_threshold
)

if fraction == 1.0:
    group.execute(plan, wait=True)
else:
    print("Path planning failed")

# Clean up
moveit_commander.roscpp_shutdown()

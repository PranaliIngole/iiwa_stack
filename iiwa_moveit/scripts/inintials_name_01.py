#!/usr/bin/env python3

import rospy
import moveit_commander
import threading
import copy
import sys
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import RobotTrajectory
from six.moves import input
from iiwa_msgs.srv import SetSmartServoJointSpeedLimits



rospy.init_node('moveit_cartesian_path_example', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

rospy.wait_for_service('/iiwa/configuration/setSmartServoLimits')

try:

    set_limits_proxy = rospy.ServiceProxy('/iiwa/configuration/setSmartServoLimits', SetSmartServoJointSpeedLimits)

    
    response = set_limits_proxy(joint_relative_velocity=0.04)

    
    rospy.loginfo("Service response:\nSuccess: %s\nError: %s", response.success, response.error)
    

except rospy.ServiceException as e:
    rospy.logerr("Service call failed: %s", str(e))

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

# Define the initial pose
initial_pose = group.get_current_pose().pose
print(initial_pose)
#make changes in waypoints only
# Define the waypoints for the "P" letter
waypoints = []

# Define the Cartesian path for the "P" letter
#keep z value fixed, change x and y within 0.2
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
    0.0,   # jump_threshold
)

if fraction == 1.0:
    # group.execute(plan, wait=True)
    # Convert the MoveIt plan to a moveit_msgs.msg.RobotTrajectory message      i
    robot_trajectory = RobotTrajectory()
    robot_trajectory.joint_trajectory = plan.joint_trajectory

    # Adjust the trajectory timing based on velocity scaling
    velocity_scaling_factor = 0.09  # Adjust this value as desired
    retime_trajectory = group.retime_trajectory(robot.get_current_state(), robot_trajectory, velocity_scaling_factor)
    input("============ Press `Enter` to execute a saved path ...")
    # print("Retime Trajectory: ",retime_trajectory)
    group.execute(retime_trajectory,wait=True)
    group.stop()
    group.clear_pose_targets()
else:
    print("Path planning failed")

# Clean up
moveit_commander.roscpp_shutdown()

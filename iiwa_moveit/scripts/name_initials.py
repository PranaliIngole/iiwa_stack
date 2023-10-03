#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
from geometry_msgs.msg import Pose, Point

# Initialize ROS node
rospy.init_node("alphabet_drawer")

# Initialize moveit_commander
moveit_commander.roscpp_initialize(sys.argv)
scene = moveit_commander.PlanningSceneInterface()
# Initialize the robot's MoveGroupCommander
robot = moveit_commander.MoveGroupCommander("manipulator")  # Replace with your robot's group name
print("nbdsfjk")
start_x = 2
start_y = 1
start_z = 1

# Create a Pose message with position
pose = Pose()
position = Point()
position.x = start_x
position.y = start_y
position.z = start_z
pose.position = position

# Waypoints for drawing the letter "P"
alphabet_P_trajectory = [
    # Starting point (Move to the initial position)
    Pose(x=start_x, y=start_y, z=start_z),

    # Draw the vertical line of "P"
    Pose(x=start_x, y=start_y + 0.1, z=start_z),  # Adjust the y-coordinate as needed
    Pose(x=start_x, y=start_y + 0.2, z=start_z),

    # Transition to the curved part
    Pose(x=start_x + 0.05, y=start_y + 0.2, z=start_z),  # Adjust the x-coordinate as needed
    Pose(x=start_x + 0.1, y=start_y + 0.15, z=start_z),  # Adjust the x and y-coordinates as needed

    # Draw the curve (quarter circle)
    # You can calculate additional waypoints to approximate the curve
    Pose(x=start_x + 0.15, y=start_y + 0.1, z=start_z),
    Pose(x=start_x + 0.2, y=start_y + 0.05, z=start_z),
    Pose(x=start_x + 0.25, y=start_y, z=start_z),

    # Ending point (Move away from the drawing)
    Pose(x=end_x, y=end_y, z=end_z)
]
# Specify the end-effector's pose tolerance
robot.set_goal_tolerance(0.01)  # Adjust as needed

# Plan and execute the trajectory
fraction = 0.0
while fraction < 1.0:
    (plan, fraction) = robot.compute_cartesian_path(
        waypoints, 0.01, 0.0, avoid_collisions=True
    )

# Execute the trajectory
robot.execute(plan)


# Loop through the trajectory points and send them to the robot
for waypoint in alphabet_P_trajectory:
    command_publisher.publish(waypoint)
    rospy.sleep(1)  # Adjust the sleep duration for timing

# Close the communication and shut down the node
rospy.signal_shutdown('Drawing complete')

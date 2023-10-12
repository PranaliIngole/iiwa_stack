#!/usr/bin/env python3

import rospy
import moveit_commander
import math
import copy
import geometry_msgs.msg
from moveit_msgs.msg import RobotTrajectory
def draw_circle():
    rospy.init_node('draw_circle')
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander("manipulator")  # Replace with your robot's group name
    move_group.set_pose_reference_frame("world")
    move_group.set_start_state_to_current_state()
    # Define circle parameters (adjust as needed)
    circle_radius = 0.2  # Radius of the circle
    num_points = 80 # Number of points on the circle (adjust for smoother or coarser circle)

    # Create a list of waypoints for the circular path
    waypoints = []
    pose = geometry_msgs.msg.Pose()
    curr=move_group.get_current_pose()
    pose.position=curr.pose.position
  
    for i in range(num_points):
        # Calculate the position for each point on the circle
        theta = 2.0 * math.pi * float(i) / float(num_points)
        x = circle_radius * math.cos(theta)
        y = circle_radius * math.sin(theta)
        z = 1.0  # Adjust the z-coordinate as needed

        # Create a Pose for the current waypoint
        
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        waypoints.append(copy.deepcopy(pose))
        print(x,y,z)
    # Plan and execute the trajectory
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        0.001,  # eef_step
        0.0,   # jump_threshold
        avoid_collisions=True
    )
    # print(len(plan.joint_trajectory.points))
    # move_group.execute(plan, wait=True)
    # Convert the MoveIt plan to a moveit_msgs.msg.RobotTrajectory message
    robot_trajectory = RobotTrajectory()
    robot_trajectory.joint_trajectory = plan.joint_trajectory

    # Adjust the trajectory timing based on velocity scaling
    velocity_scaling_factor = 0.09  # Adjust this value as desired
    retime_trajectory = move_group.retime_trajectory(robot.get_current_state(), robot_trajectory, velocity_scaling_factor)
    # print("Retime Trajectory: ",retime_trajectory)
    move_group.execute(retime_trajectory,wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    # Define the home joint configuration 
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0  # 1/6 of a turn
    joint_goal[6] = 0


    # Plan and execute the motion to the home position
    move_group.go(joint_goal, wait=True)
    
if __name__ == '__main__':
    try:
        draw_circle()
    except rospy.ROSInterruptException:
        pass

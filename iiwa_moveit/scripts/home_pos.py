#!/usr/bin/env python3

import rospy
import moveit_commander
import math
def move_to_home():
    rospy.init_node('move_to_home')
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("manipulator")  # Replace with your robot's group name

    # Define the home joint configuration (modify this according to your robot)
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0  # 1/6 of a turn
    joint_goal[6] = 0

    # Set the target joint configuration
    #group.set_joint_value_target(joint_goal)

    # Plan and execute the motion to the home position
    group.go(joint_goal, wait=True)

if __name__ == '__main__':
    try:
        move_to_home()
    except rospy.ROSInterruptException:
        pass


    

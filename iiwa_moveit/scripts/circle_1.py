import rospy
import moveit_commander
import math
import copy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def execute_circular_trajectory_in_gazebo(planned_trajectory):
    rospy.init_node('execute_circular_trajectory_in_gazebo')
    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander("manipulator")  # Replace with your robot's group name
    pub = rospy.Publisher('/your_robot/joint_trajectory_controller/command', JointTrajectory, queue_size=10)

    # Create a JointTrajectory message
    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = move_group.get_active_joints()

    # Add the trajectory points from the planned trajectory
    for point in planned_trajectory.joint_trajectory.points:
        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = point.positions
        joint_trajectory_point.velocities = point.velocities
        joint_trajectory_point.time_from_start = point.time_from_start
        joint_trajectory.points.append(joint_trajectory_point)

    # Publish the JointTrajectory to Gazebo
    pub.publish(joint_trajectory)

def draw_circle():
    # Define circle parameters (adjust as needed)
    circle_radius = 0.1  # Radius of the circle
    num_points = 50  # Number of points on the circle (adjust for smoother or coarser circle)

    # Create a list of waypoints for the circular path
    waypoints = []

    for i in range(num_points):
        # Calculate the position for each point on the circle
        theta = 2.0 * math.pi * float(i) / float(num_points)
        x = circle_radius * math.cos(theta)
        y = circle_radius * math.sin(theta)
        z = 0.1  # Adjust the z-coordinate as needed

        # Create a Pose for the current waypoint
        pose = geometry_msgs.msg.Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        waypoints.append(copy.deepcopy(pose))

    # Plan and execute the trajectory
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        0.01,  # eef_step
        0.0,   # jump_threshold
        avoid_collisions=True
    )

    move_group.execute(plan, wait=True)

if __name__ == '__main__':
    try:
        # Plan the circular trajectory using the modified draw_circle function
        planned_trajectory = draw_circle()

        # Execute the circular trajectory in Gazebo
        execute_circular_trajectory_in_gazebo(planned_trajectory)
    except rospy.ROSInterruptException:
        pass

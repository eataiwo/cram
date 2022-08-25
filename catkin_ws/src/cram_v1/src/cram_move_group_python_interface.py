import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("cram_move_group_python_interface", anonymous=True)

widowx = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "widowx_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path",
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = widowx.get_group_names()
print("============ Available Planning Groups:", widowx.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(widowx.get_current_state())
print("")

# Planning to a Joint Goal
# We get the joint values from the group and change some of the values:
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] = 0
joint_goal[3] = 0

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
move_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()

# Planning to a Pose Goal
# We can plan a motion for this group to a desired pose for the end-effector:
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.1
pose_goal.position.y = 0.05
pose_goal.position.z = 0.3

move_group.set_pose_target(pose_goal)

# `go()` returns a boolean indicating whether the planning and execution was successful.
success = move_group.go(wait=True)
print(f"The planner was successful: {success}")

# Calling `stop()` ensures that there is no residual movement
move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets().
move_group.clear_pose_targets()

waypoints = []
scale=0.1
#Cartesian Paths
# You can plan a Cartesian path directly by specifying a list of waypoints for the end-effector to go through.
wpose = move_group.get_current_pose().pose
wpose.position.z -= scale * 0.5  # First move up (z)
wpose.position.y += scale * 0.5  # and sideways (y)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x += scale * 0.5  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.y -= scale * 0.5  # Third move sideways (y)
waypoints.append(copy.deepcopy(wpose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient
# for this tutorial.
(plan, fraction) = move_group.compute_cartesian_path(
    waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
)  # jump_threshold

# Note: We are just planning, not asking move_group to actually move the robot yet:
return plan, fraction


# Displaying a Trajectory
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
# Publish
display_trajectory_publisher.publish(display_trajectory)

# Executing a Plan
move_group.execute(plan, wait=True)


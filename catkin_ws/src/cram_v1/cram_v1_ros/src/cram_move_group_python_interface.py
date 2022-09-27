import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import tf2_ros
import tf2_geometry_msgs
import tf
from tf import transformations as ts

from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

### Setup - init #######
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("cram_move_group_python_interface", anonymous=True)

tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
tf_listener = tf2_ros.TransformListener(tf_buffer)

# Commander
cram_rig = moveit_commander.RobotCommander()

# Planner
scene = moveit_commander.PlanningSceneInterface()

# Move group
widowx_1 = moveit_commander.MoveGroupCommander("widowx_1")
widowx_2 = moveit_commander.MoveGroupCommander("widowx_2")
all_arms = moveit_commander.MoveGroupCommander("cram_rig")

#
display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path",
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

####
widowx_1.set_max_velocity_scaling_factor = 1
widowx_2.set_max_velocity_scaling_factor = 1


#### Setting other planner parameters

widowx_1.set_goal_position_tolerance(0.05)
widowx_2.set_goal_position_tolerance(0.05)
widowx_1.set_goal_orientation_tolerance(0.05)
widowx_2.set_goal_orientation_tolerance(0.05)
widowx_1.set_goal_joint_tolerance(0.05)
widowx_2.set_goal_joint_tolerance(0.05)
widowx_1.set_goal_tolerance(0.05)
widowx_2.set_goal_tolerance(0.05)

##### VERBOSE #######
# We can get the name of the reference frame for this robot:
widowx_1_planning_frame = widowx_1.get_planning_frame()
widowx_2_planning_frame = widowx_2.get_planning_frame()
all_arms_planning_frame = all_arms.get_planning_frame()
print("============ Planning frame widowx_1: %s" % widowx_1_planning_frame)
print("============ Planning frame widowx_2: %s" % widowx_2_planning_frame)
print("============ Planning frame all_arms: %s" % all_arms_planning_frame)

widowx_1_pose_reference_frame = widowx_1.get_pose_reference_frame()
widowx_2_pose_reference_frame = widowx_2.get_pose_reference_frame()
all_arms_pose_reference_frame = all_arms.get_pose_reference_frame()
print("============ pose_reference_frame widowx_1: %s" % widowx_1_planning_frame)
print("============ pose_reference_frame widowx_2: %s" % widowx_2_planning_frame)
print("============ pose_reference_frame all_arms: %s" % all_arms_planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = widowx_1.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
cram_rig_group_names = cram_rig.get_group_names()
print("============ Available Planning Groups:", cram_rig_group_names)

#### Saved goal states
# widowx_1_saved_joint_states = widowx_1.get_remembered_joint_values()
# widowx_2_saved_joint_states = widowx_2.get_remembered_joint_values()
# all_arms_saved_joint_states = all_arms.get_remembered_joint_values()
# print("============ Available saved poses widowx 1:", widowx_1_saved_joint_states)
# print("============ Available saved poses widowx 2:", widowx_2_saved_joint_states)
# print("============ Available saved poses all arms:", all_arms_saved_joint_states)


# # Sometimes for debugging it is useful to print the entire state of the
# # robot:
# print("============ Printing robot state")
# print(cram_rig.get_current_state())
# print("")

###### Home ########

# Planning to a Joint Goal
# We get the joint values from the group and change some of the values:

# List of commonly used joint states
home_goal = [0, -pi/2, pi/2, 0]
print_home_vertical = [0, 0.46, 1.08913, -0.05522]
print_home_horizontal = (0, 0.925, 0.81301, -1.7303)

widowx_1.remember_joint_values("print_home_vertical", print_home_vertical)
widowx_1.remember_joint_values("home", home_goal)

widowx_2.remember_joint_values("home", home_goal)
all_arms.remember_joint_values("home", home_goal+home_goal)


# widowx_1_joint_goal = widowx_1.get_current_joint_values()
# widowx_2_joint_goal = widowx_2.get_current_joint_values()
# all_arms_joint_goal = all_arms.get_current_joint_values()
# widowx_1_joint_goal = home_goal
# widowx_2_joint_goal = home_goal
# all_arms_joint_goal = home_goal + home_goal


all_arms.set_named_target("home")


# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
# widowx_1.go(widowx_1_joint_goal, wait=True)
# widowx_2.go(widowx_2_joint_goal, wait=True)
all_arms.go(wait=True)


# Calling ``stop()`` ensures that there is no residual movement
# widowx_1.stop()
# widowx_2.stop()
all_arms.stop()
print("=========================================")
print(f'Home pose is:\n{widowx_1.get_current_pose()}')

#
# widowx_1.set_goal_tolerance(0.5)
# widowx_1.set
# widowx_1.set
# widowx_1.set
# widowx_1.set


##########   ##########
# Planning to a Pose Goal ######### Make function

widowx_1_current_pose = widowx_1.get_current_pose()

widowx_1.set_named_target("print_home_vertical")
widowx_1.go(wait=True)
widowx_1.stop()
widowx_1.clear_pose_targets()

print("=========================================")
print(f'Current pose is:\n{widowx_1.get_current_pose()}')

# We can plan a motion for this group to a desired pose for the end-effector:
pose_goal = widowx_1_current_pose
# pose_goal.pose.orientation.x = 1e-6
# pose_goal.pose.orientation.y = 1e-6
# pose_goal.pose.orientation.z = 1e-6
# pose_goal.pose.orientation.w = 1.000000

# pose_goal.pose.position.x = round(pose_goal.pose.position.x, 6)
# pose_goal.pose.position.y += -0.200000
pose_goal.header.stamp = rospy.Time.now()
# pose_goal.pose.position.z = round(pose_goal.pose.position.z, 6)


widowx_1.set_pose_target(pose_goal)
print(f'Goal pose is:\n{pose_goal}')

# `go()` returns a boolean indicating whether the planning and execution was successful.
success = widowx_1.go(wait=True)
print(f"The planner was successful: {success}")

# Calling `stop()` ensures that there is no residual movement
widowx_1.stop()
widowx_1.clear_pose_targets()


# print("=========================================")
#
# # We can plan a motion for this group to a desired pose for the end-effector:
# pose_goal = geometry_msgs.msg.PoseStamped()
# pose_goal.header.stamp = rospy.Time.now()
# pose_goal.pose.orientation.x = -0.5
# pose_goal.pose.orientation.y = 0.5
# pose_goal.pose.orientation.z = 0.5
# pose_goal.pose.orientation.w = 0.5
#
# pose_goal.pose.position.x = 0.2
# pose_goal.pose.position.y = 0.2
# pose_goal.pose.position.z = 0.3
#
#
# widowx_1.set_pose_target(pose_goal.pose)
# print("=========================================")
# print(f'Current pose is:\n{widowx_1_home_pose}')
# print(f'Goal pose is:\n{pose_goal.pose}')
# print("=========================================")
#
# # `go()` returns a boolean indicating whether the planning and execution was successful.
# success = widowx_1.go(wait=True)
# print(f"The planner was successful: {success}")
#
# # Calling `stop()` ensures that there is no residual movement
# widowx_1.stop()
# # It is always good to clear your targets after planning with poses.
# # Note: there is no equivalent function for clear_joint_value_targets().
# widowx_1.clear_pose_targets()
#
#
#
# try:
#     transform = tf_buffer.lookup_transform('base', 'buildplate', rospy.Time(0))
# except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#     pass
#
# # transform = tf_buffer.lookup_transform('base', 'buildplate', rospy.Time())
# # object_pose = PoseStamped()
# # object_pose.header.stamp = rospy.Time.now()
# # object_pose.pose.position = pose_goal.pose.posistion
#
#
# trans_pose_goal = tf2_geometry_msgs.do_transform_pose(pose_goal, transform)
# print("=========================================")
# print(f'Current pose is:{trans_pose_goal.pose}')
# widowx_1.set_pose_target(trans_pose_goal.pose)
#
# # `go()` returns a boolean indicating whether the planning and execution was successful.
# success = widowx_1.go(wait=True)
# print("=========================================")
# print(f'Current pose is:\n{widowx_1_home_pose}')
# print(f'Goal pose is:\n{pose_goal.pose}')
# print("=========================================")
# print(f"The planner was successful: {success}")
#
# # Calling `stop()` ensures that there is no residual movement
# widowx_1.stop()
# # It is always good to clear your targets after planning with poses.
# # Note: there is no equivalent function for clear_joint_value_targets().
# widowx_1.clear_pose_targets()




# #Cartesian Paths ######## Make function
# waypoints = []
# scale=0.1
# # You can plan a Cartesian path directly by specifying a list of waypoints for the end-effector to go through.
# wpose = move_group.get_current_pose().pose
# wpose.position.z -= scale * 0.5  # First move up (z)
# wpose.position.y += scale * 0.5  # and sideways (y)
# waypoints.append(copy.deepcopy(wpose))
#
# wpose.position.x += scale * 0.5  # Second move forward/backwards in (x)
# waypoints.append(copy.deepcopy(wpose))
#
# wpose.position.y -= scale * 0.5  # Third move sideways (y)
# waypoints.append(copy.deepcopy(wpose))
#
# # We want the Cartesian path to be interpolated at a resolution of 1 cm
# # which is why we will specify 0.01 as the eef_step in Cartesian
# # translation.  We will disable the jump threshold by setting it to 0.0,
# # ignoring the check for infeasible jumps in joint space, which is sufficient
# # for this tutorial.
# (plan, fraction) = move_group.compute_cartesian_path(
#     waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
# )  # jump_threshold
#
# # Note: We are just planning, not asking move_group to actually move the robot yet:
# return plan, fraction
#
# # Displaying a Trajectory
# display_trajectory = moveit_msgs.msg.DisplayTrajectory()
# display_trajectory.trajectory_start = robot.get_current_state()
# display_trajectory.trajectory.append(plan)
# # Publish
# display_trajectory_publisher.publish(display_trajectory)
#
# # Executing a Plan
# move_group.execute(plan, wait=True)


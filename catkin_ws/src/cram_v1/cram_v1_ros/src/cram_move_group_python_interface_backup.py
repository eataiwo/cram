#!/usr/bin/env python

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
# rospy.set_param('/widowx_1/position_only_ik', False)

tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
tf_listener = tf2_ros.TransformListener(tf_buffer)

#
display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path",
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

# Commander
cram_rig = moveit_commander.RobotCommander()

# Planner
scene = moveit_commander.PlanningSceneInterface()

# Move group
widowx_1 = moveit_commander.MoveGroupCommander("widowx_1")
widowx_2 = moveit_commander.MoveGroupCommander("widowx_2")

# Setting other planner parameters
widowx_1.set_max_velocity_scaling_factor = 1
# widowx_1.set_goal_position_tolerance(0.1)
# widowx_1.set_goal_orientation_tolerance(0.1)
# widowx_1.set_goal_tolerance(0.1)
# widowx_1.set_goal_joint_tolerance(0.1)
widowx_1.set_num_planning_attempts(8)
widowx_1.set_planning_time(2)
# widowx_1.set_pose_reference_frame("base")
# widowx_1.set_pose_reference_frame("widowx_1_arm_base_link")

# widowx_1.set_planner_id("RRTConnectkConfigDefault")

# Second Arm ######
widowx_2.set_goal_position_tolerance(0.05)
widowx_2.set_goal_orientation_tolerance(0.05)
widowx_2.set_goal_joint_tolerance(0.05)
widowx_2.set_goal_tolerance(0.05)
widowx_2.set_max_velocity_scaling_factor = 1
widowx_2.set_num_planning_attempts(3)

# VERBOSE #######
# We can get the name of the reference frame for this robot:
widowx_1_planning_frame = widowx_1.get_planning_frame()
widowx_2_planning_frame = widowx_2.get_planning_frame()
print("\n")
print("=====================================================")
print("============ Planning frame widowx_1: %s ============" % widowx_1_planning_frame)
print("============ Planning frame widowx_2: %s ============" % widowx_2_planning_frame)
print("=====================================================")
print("\n")

widowx_1_pose_reference_frame = widowx_1.get_pose_reference_frame()
widowx_2_pose_reference_frame = widowx_2.get_pose_reference_frame()
print("=====================================================")
print("============ pose_reference_frame widowx_1: %s ============" % widowx_1_pose_reference_frame)
print("============ pose_reference_frame widowx_2: %s ============" % widowx_2_pose_reference_frame)
print("=====================================================")
print("\n")

# We can also print the name of the end-effector link for this group:
eef_link = widowx_1.get_end_effector_link()
print("=====================================================")
print("============ End effector link: %s ============" % eef_link)
print("=====================================================")
print("\n")

# We can get a list of all the groups in the robot:
cram_rig_group_names = cram_rig.get_group_names()
print("============ Available Planning Groups: %s ============", cram_rig_group_names)
print("=====================================================")
print("\n")

# List of commonly used joint states
home_goal = [0, -pi / 2, pi / 2, 0]
print_home_vertical = [0, 0.46, 1.08913, -0.05522]
print_home_horizontal = (0, 0.925, 0.81301, -1.7303)

# Remember States
widowx_1.remember_joint_values("print_idle_vertical", print_home_vertical)
widowx_1.remember_joint_values("home", home_goal)

widowx_2.remember_joint_values("home", home_goal)

# Setting target using named targer previously set
widowx_1.set_named_target("home")
widowx_2.set_named_target("home")

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
widowx_1.go(wait=True)
widowx_2.go(wait=True)

# Calling ``stop()`` ensures that there is no residual movement
widowx_1.stop()
widowx_2.stop()

print("\n")
print("=====================================================")
print(f'Home pose is:\n{widowx_1.get_current_pose()}')
print("=====================================================")
print("\n")

# Saving current pose
widowx_1_home_pose = widowx_1.get_current_pose()
rospy.sleep(0.2)

# Go to print vertical home
# Planning to a Pose Goal ######### Make function eventually
widowx_1.set_named_target("print_idle_vertical")
widowx_1.go(wait=True)
widowx_1.stop()
widowx_1.clear_pose_targets()

print("\n")
print("=====================================================")
print(f'Print pose is:\n{widowx_1.get_current_pose()}')
print("=====================================================")
print("\n")

widowx_1_print_pose = widowx_1.get_current_pose()
rospy.sleep(0.2)

# Return back to home pose
widowx_1.set_named_target("home")
widowx_1.go(wait=True)
widowx_1.stop()

# Using shift pose target
# widowx_1.shift_pose_target(6, 0.1)
# widowx_1.go(wait=True)
# widowx_1.stop()
# rospy.sleep(5)

print("\n")
print("=====================================================")
print(f'Current pose is:\n{widowx_1.get_current_pose()}')
print("=====================================================")
print("\n")

pose_goal = geometry_msgs.msg.PoseStamped()
transform = tf_buffer.lookup_transform('widowx_1_arm_base_link', 'base', rospy.Time())
pose_goal.header.stamp = rospy.Time.now()
# pose_goal = widowx_1_home_pose
# pose_goal = tf2_geometry_msgs.do_transform_pose(widowx_1_print_pose, transform)
#
# print("\n")
# print("=====================================================")
# print(f'TF Goal pose is:\n{pose_goal.pose}')
# print("=====================================================")
# print("\n")


# pose_goal.pose.position.x = round(1e-6, 6)
# pose_goal.pose.position.y = round(1e-6, 6)
# pose_goal.pose.position.z = round(pose_goal.pose.position.z, 6)
# pose_goal.pose.orientation.x = round(pose_goal.pose.orientation.x, 6)
# pose_goal.pose.orientation.y = round(pose_goal.pose.orientation.y, 6)
# pose_goal.pose.orientation.z = round(pose_goal.pose.orientation.z, 6)
# pose_goal.pose.orientation.w = round(pose_goal.pose.orientation.w, 6)

print("\n")
print("=====================================================")
print(f'Raw Goal pose is:\n{pose_goal}')
print("=====================================================")
print("\n")

transform = tf_buffer.lookup_transform('base', 'base', rospy.Time(0))
pose_goal = tf2_geometry_msgs.do_transform_pose(pose_goal, transform)
# pose_goal.header.stamp = rospy.Time.now()

pose_goal.header.stamp = rospy.Time.now()
pose_goal.pose.orientation.x = 0.000001000
pose_goal.pose.orientation.y = 0.707106800
pose_goal.pose.orientation.z = 0.000001000
pose_goal.pose.orientation.w = 0.707106800
pose_goal.pose.position.x = 0.25000111 - 0.245
pose_goal.pose.position.y = 0.00000111
pose_goal.pose.position.z = 0.25000111 + 0.04

pose_goal.pose.position.x = round(pose_goal.pose.position.x, 8)
pose_goal.pose.position.y = round(pose_goal.pose.position.y, 8)
pose_goal.pose.position.z = round(pose_goal.pose.position.z, 8)
pose_goal.pose.orientation.x = round(pose_goal.pose.orientation.x, 8)
pose_goal.pose.orientation.y = round(pose_goal.pose.orientation.y, 8)
pose_goal.pose.orientation.z = round(pose_goal.pose.orientation.z, 8)
pose_goal.pose.orientation.w = round(pose_goal.pose.orientation.w, 8)

# ## Rotating just the base
# q = [pose_goal.pose.orientation.x, pose_goal.pose.orientation.y, pose_goal.pose.orientation.z, pose_goal.pose.orientation.w]
#
# print("\n")
# print("=====================================================")
# [r, p, y] = tf.transformations.euler_from_quaternion(q)
# print(f'{r} {p} {y}')
# print("=====================================================")
# print("\n")
# y = 1e-6
# q = tf.transformations.quaternion_from_euler(r, p, y)
# pose_goal.pose.orientation.x = round(q[0], 6)
# pose_goal.pose.orientation.y = round(q[1], 6)
# pose_goal.pose.orientation.z = round(q[2], 6)
# pose_goal.pose.orientation.w = round(q[3], 6)

print("\n")
print("=====================================================")
print(f'Rounded TF Goal pose is:\n{pose_goal}')
print("=====================================================")
print("\n")

widowx_1.set_pose_target(pose_goal)

success = False
i = 1
while not success:
    # `go()` returns a boolean indicating whether the planning and execution was successful.
    success = widowx_1.go(wait=True)
    print(f"Attempt number {i}. The planner was successful: {success}")
    i += 1
    rospy.sleep(1)

# Calling `stop()` ensures that there is no residual movement
widowx_1.stop()
widowx_1.clear_pose_targets()

#####################################################

pose_goal = geometry_msgs.msg.PoseStamped()
transform = tf_buffer.lookup_transform('base', 'base', rospy.Time(0))
pose_goal = tf2_geometry_msgs.do_transform_pose(pose_goal, transform)
pose_goal.header.stamp = rospy.Time.now()
pose_goal.pose.orientation.x = 0.000001000
pose_goal.pose.orientation.y = 1.000001000
pose_goal.pose.orientation.z = 0.000001000
pose_goal.pose.orientation.w = 0.000001000
pose_goal.pose.position.x = 0.20000111 - 0.245
pose_goal.pose.position.y = 0.00000111
pose_goal.pose.position.z = 0.20000111 + 0.04

pose_goal.pose.position.x = round(pose_goal.pose.position.x, 8)
pose_goal.pose.position.y = round(pose_goal.pose.position.y, 8)
pose_goal.pose.position.z = round(pose_goal.pose.position.z, 8)
pose_goal.pose.orientation.x = round(pose_goal.pose.orientation.x, 8)
pose_goal.pose.orientation.y = round(pose_goal.pose.orientation.y, 8)
pose_goal.pose.orientation.z = round(pose_goal.pose.orientation.z, 8)
pose_goal.pose.orientation.w = round(pose_goal.pose.orientation.w, 8)

widowx_1.set_pose_target(pose_goal)

success = False
i = 1
while not success:
    # `go()` returns a boolean indicating whether the planning and execution was successful.
    success = widowx_1.go(wait=True)
    print(f"Attempt number {i}. The planner was successful: {success}")
    i += 1
    rospy.sleep(1)

# Calling `stop()` ensures that there is no residual movement
widowx_1.stop()
widowx_1.clear_pose_targets()

#####################################################

pose_goal = geometry_msgs.msg.PoseStamped()
transform = tf_buffer.lookup_transform('base', 'base', rospy.Time(0))
pose_goal = tf2_geometry_msgs.do_transform_pose(pose_goal, transform)
pose_goal.header.stamp = rospy.Time.now()
pose_goal.pose.orientation.x = 0.000001000
pose_goal.pose.orientation.y = 1.000001000
pose_goal.pose.orientation.z = 0.000001000
pose_goal.pose.orientation.w = 0.000001000
pose_goal.pose.position.x = 0.20000111 - 0.245
pose_goal.pose.position.y = 0.00000111
pose_goal.pose.position.z = 0.10000111 + 0.04

pose_goal.pose.position.x = round(pose_goal.pose.position.x, 8)
pose_goal.pose.position.y = round(pose_goal.pose.position.y, 8)
pose_goal.pose.position.z = round(pose_goal.pose.position.z, 8)
pose_goal.pose.orientation.x = round(pose_goal.pose.orientation.x, 8)
pose_goal.pose.orientation.y = round(pose_goal.pose.orientation.y, 8)
pose_goal.pose.orientation.z = round(pose_goal.pose.orientation.z, 8)
pose_goal.pose.orientation.w = round(pose_goal.pose.orientation.w, 8)

widowx_1.set_pose_target(pose_goal)

success = False
i = 1
while not success:
    # `go()` returns a boolean indicating whether the planning and execution was successful.
    success = widowx_1.go(wait=True)
    print(f"Attempt number {i}. The planner was successful: {success}")
    i += 1
    rospy.sleep(1)

# Calling `stop()` ensures that there is no residual movement
widowx_1.stop()
widowx_1.clear_pose_targets()

#####################################################

pose_goal = geometry_msgs.msg.PoseStamped()
transform = tf_buffer.lookup_transform('base', 'base', rospy.Time(0))
pose_goal = tf2_geometry_msgs.do_transform_pose(pose_goal, transform)
pose_goal.header.stamp = rospy.Time.now()
pose_goal.pose.orientation.x = 0.000001000
pose_goal.pose.orientation.y = 1.000001000
pose_goal.pose.orientation.z = 0.000001000
pose_goal.pose.orientation.w = 0.000001000
pose_goal.pose.position.x = 0.10000111 - 0.245
pose_goal.pose.position.y = 0.00000111
pose_goal.pose.position.z = 0.10000111 + 0.04

pose_goal.pose.position.x = round(pose_goal.pose.position.x, 8)
pose_goal.pose.position.y = round(pose_goal.pose.position.y, 8)
pose_goal.pose.position.z = round(pose_goal.pose.position.z, 8)
pose_goal.pose.orientation.x = round(pose_goal.pose.orientation.x, 8)
pose_goal.pose.orientation.y = round(pose_goal.pose.orientation.y, 8)
pose_goal.pose.orientation.z = round(pose_goal.pose.orientation.z, 8)
pose_goal.pose.orientation.w = round(pose_goal.pose.orientation.w, 8)

widowx_1.set_pose_target(pose_goal)

success = False
i = 1
while not success:
    # `go()` returns a boolean indicating whether the planning and execution was successful.
    success = widowx_1.go(wait=True)
    print(f"Attempt number {i}. The planner was successful: {success}")
    i += 1
    rospy.sleep(1)

# Calling `stop()` ensures that there is no residual movement
widowx_1.stop()
widowx_1.clear_pose_targets()

# Ideal pose reached for 3d printing in the arm vertical position

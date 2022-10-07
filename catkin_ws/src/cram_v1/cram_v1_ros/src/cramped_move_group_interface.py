#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
from tf import transformations as ts
from math import pi, tau, dist, fabs, cos
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

from moveit_commander.conversions import pose_to_list


def round_pose_values(pose, sigfig=6):
    """
    Round pose values
    """
    pose.position.x = round(pose.position.x, sigfig)
    pose.position.y = round(pose.position.y, sigfig)
    pose.position.z = round(pose.position.z, sigfig)
    pose.orientation.x = round(pose.orientation.x, sigfig)
    pose.orientation.y = round(pose.orientation.y, sigfig)
    pose.orientation.z = round(pose.orientation.z, sigfig)
    pose.orientation.w = round(pose.orientation.w, sigfig)

    return pose


def array_to_pose(pose_array=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):

    pose = Pose()
    pose.position.x = pose_array[0]
    pose.position.y = pose_array[1]
    pose.position.z = pose_array[2]
    pose.orientation.x = pose_array[3]
    pose.orientation.y = pose_array[4]
    pose.orientation.z = pose_array[5]
    pose.orientation.w = pose_array[6]
    pose = round_pose_values(pose)
    return pose


class MoveitInterface:
    def __init__(self, group_name="widowx_1", verbose=False,
                 start_state=None):
        """
        Initialises relevant MoveIt things, sets up ROS interfaces, and go to an initial ready pose.
        """
        # Common joint states for a 4DOF arm
        self.print_home_vertical = [0, 0.46, 1.08913, -0.05522]
        self.print_home_horizontal = [0, 0.925, 0.81301, -1.7303]
        self.home_goal = [0, -pi / 2, pi / 2, 0]

        # Initialise moveit_commander and a rospy node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_interface", anonymous=False)

        # Initialise a RobotCommander object. Provides information such as
        # the robot’s kinematic model and the robot’s current joint states
        self.robot = moveit_commander.RobotCommander()

        # Initialise a PlanningSceneInterface object. This provides a remote interface
        # for getting, setting, and updating the robot’s internal
        # understanding of the surrounding world:
        self.scene = moveit_commander.PlanningSceneInterface()

        # Initialise a MoveGroupCommander object. This object is an interface to a
        # planning group (group of joints).This interface can be used to plan and
        # execute motions:
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # Initialise a tf buffer and a transform listener.
        # This will report back transforms between different frames
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(60.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz:
        # display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path",
        #                                                moveit_msgs.msg.DisplayTrajectory,
        #                                                queue_size=20)

        # Move robot arm to start state
        if start_state is None:
            start_state = [0, -pi / 2, pi / 2, 0]

        self.move_group.set_joint_value_target(start_state)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        if verbose:
            # Get the name of the reference frame for this robot:
            planning_frame = self.move_group.get_planning_frame()
            print("\n")
            print("=====================================================")
            print(f"=== Planning frame of {group_name}: {planning_frame} ===")
            print("=====================================================")
            print("\n")

            # Get the name of the pose reference frame for this robot:
            pose_reference_frame = self.move_group.get_pose_reference_frame()
            print("=====================================================")
            print(f"=== Pose reference frame of {group_name}: {pose_reference_frame} ===")
            print("=====================================================")
            print("\n")

            # Get the name of the end-effector link for this group:
            eef_link = self.move_group.get_end_effector_link()
            print("=====================================================")
            print(f"=== End effector link for {group_name}: {eef_link} ===")
            print("=====================================================")
            print("\n")

            # Get a list of all the groups in the robot:
            group_names = self.move_group.get_group_names()
            print(f"=== Available Planning Groups: {group_names} ===")
            print("=====================================================")
            print("\n")

    def setup(self, max_velocity_scaling_factor=1, max_acceleration_scaling_factor=1,
              set_num_planning_attempts=4, set_planning_time=1, workspace=[]):
        """
        Modify the default parameters for moveit
        """

        self.move_group.set_max_velocity_scaling_factor = max_velocity_scaling_factor
        self.move_group.set_max_acceleration_scaling_factor = max_acceleration_scaling_factor
        self.move_group.set_num_planning_attempts(set_num_planning_attempts)
        self.move_group.set_planning_time(set_planning_time)
        self.move_group.set_workspace(workspace)
        # self.move_group.set_goal_position_tolerance(0.1)
        # self.move_group.set_goal_orientation_tolerance(0.1)
        # self.move_group.set_goal_tolerance(0.1)
        # self.move_group.set_goal_joint_tolerance(0.1)
        # self.move_group.set_pose_reference_frame("base")
        # self.move_group.set_planner_id("RRTConnectkConfigDefault")

    def joint_go(self, joint_target, wait=True):
        """
        Execute a joint target movement
        """
        self.move_group.set_joint_value_target(joint_target)
        self.move_group.go(wait)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

    def pose_go(self, pose_target, wait=True):
        """
        Execute a pose goal movement
        """
        pose_target.header.stamp = rospy.Time.now()
        pose_target.pose = round_pose_values(pose_target.pose)
        self.move_group.set_pose_target(pose_target.pose)
        success = self.move_group.go(wait)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success

    def transform_pose(self, pose, from_frame, to_frame):
        """
        Transform a pose from one frame to another
            """
        try:
            transform = self.tf_buffer.lookup_transform(from_frame, to_frame, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

        trans_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
        return trans_pose

    def cartesian_go(self, goal, start=None, wait=True):
        """
        Execute a cartesian movement
        """
        waypoints = []
        # You can plan a Cartesian path directly by specifying a
        # list of waypoints for the end-effector to go through.
        if start is None:
            wpose_start = self.move_group.get_current_pose().pose
            waypoints.append(copy.deepcopy(wpose_start))
        else:
            waypoints.append(copy.deepcopy(start))

        waypoints.append(copy.deepcopy(goal))

        # We want the Cartesian path to be interpolated at a resolution of 1 mm
        # which is why we will specify 0.001 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 1.0)  # waypoints to follow  # eef_step # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:

        # Executing a Plan
        self.move_group.execute(plan, wait)

        return fraction

    def run_demo(self):
        waypoints=[]
        home_vertical_3d_printer = []



if __name__ == "__main__":
    cramped_interface = MoveitInterface()
    cramped_interface.setup()
    cramped_interface.run_demo()

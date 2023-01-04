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

from moveit_commander.conversions import pose_to_list
from moveit_commander.conversions import list_to_pose
from moveit_commander.conversions import list_to_pose_stamped


def round_pose_values(pose, sigfig=6):
    """
    Round pose values
    """
    pose.pose.position.x = round(pose.pose.position.x, sigfig)
    pose.pose.position.y = round(pose.pose.position.y, sigfig)
    pose.pose.position.z = round(pose.pose.position.z, sigfig)
    pose.pose.orientation.x = round(pose.pose.orientation.x, sigfig)
    pose.pose.orientation.y = round(pose.pose.orientation.y, sigfig)
    pose.pose.orientation.z = round(pose.pose.orientation.z, sigfig)
    pose.pose.orientation.w = round(pose.pose.orientation.w, sigfig)

    return pose


class MoveitInterface:
    def __init__(self, group_name="widowx_1", verbose=False,
                 start_state=None):
        """
        Initialises relevant MoveIt things, sets up ROS interfaces, and go to an initial ready pose.
        """
        self.eef_step = 0.0001
        self.jump_threshold = 0.0

        # Common joint states for a 4DOF arm
        self.print_home_vertical = [0, 0.46, 1.08913, -0.05522]
        self.print_home_horizontal = [0, 0.925, 0.81301, -1.7303]
        self.home_goal = [0, -pi / 2, pi / 2, 0]

        # TODO: Add poses for home positions too as attirbutes

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
            group_names = self.robot.get_group_names()
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
        success = self.move_group.go(wait)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()
        return success

    def pose_go(self, pose_target, wait=True):
        """
        Execute a pose goal movement
        """
        # pose_target.header.stamp = rospy.Time.now()
        # pose_target.pose = round_pose_values(pose_target.pose)
        self.move_group.set_pose_target(pose_target)
        success = self.move_group.go(wait)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success

    def relative_pose_go(self, axis, value, wait=True):  #
        """
        Execute a relative pose movement. Add value to the corresponding axis
        (0..5: X, Y, Z, R, P, Y) and set the new pose as the pose target.
        Note: This does not take a quaternion but roll, pitch, yaw angles.
        """
        self.move_group.shift_pose_target(axis, value)
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

        waypoints.append(copy.deepcopy(goal.pose))
        print("=====================================================")
        print(f"=== Waypoints {waypoints} ===")
        print("=====================================================")
        print("\n")

        # We want the Cartesian path to be interpolated at a resolution of 1 mm
        # which is why we will specify 0.001 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, self.eef_step, self.jump_threshold)  # waypoints to follow  # eef_step # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)


        # Executing a Plan
        self.move_group.execute(plan, wait)

        return plan

    def go_home(self, robot_arm="widowx1"):
        # TODO: Add a check for the orientation of the end effector to make sure it was in printing mode before doing home movement
        home_pose = self.home_pose["widowx1"]
        successes = []
        wpose = self.move_group.get_current_pose().pose
        # waypoints.append(copy.deepcopy(wpose))

        # Clear the print bed or work piece by 30mm
        wpose.position.z = wpose.position.z + 0.03
        # waypoints.append(copy.deepcopy(wpose))
        success = self.cartesian_go(wpose, wait=True)
        successes.append(success)

        # Go to home co-ordinates in x and y
        wpose.position.x = home_pose.position.x
        wpose.position.y = home_pose.position.y
        # waypoints.append(copy.deepcopy(wpose))
        success = self.cartesian_go(wpose, wait=True)
        successes.append(success)

        # Finally go to home co-ordinates in z
        wpose.position.z = home_pose.position.z
        # waypoints.append(copy.deepcopy(wpose))
        success = self.cartesian_go(wpose, wait=True)
        successes.append(success)

        if all(successes):
            return True
        else:
            return False

    def run_demo(self):
        """
        Demo of the class functionality.
        """
        vertical_printing_quaternion = (0.000000, 1.000000, 0.000000, 0.000000)
        horizontal_printing_quaternion = (0.000000, 0.7071068, 0.000000, 0.7071068)

        # Change this to be the back corner of the print bed so the hot ends are far away from the user and work piece
        vertical_printing_idle = [0.10000111 - 0.245, 0.00000111, 0.10000111 + 0.04, 0.000000, 1.000000, 0.000000,
                                  0.000000]
        horizontal_printing_idle = []

        robot_arm_idle = (0, 0, 0, pi / 4)
        robot_arm_home = (0, -pi / 2, pi / 2, 0)
        waypoints = []

        # Start a robot arm home position.
        success = self.joint_go(robot_arm_home, wait=True)

        # Go to vertical printing home position
        vertical_printing_idle_pose = list_to_pose(vertical_printing_idle)
        success = self.pose_go(vertical_printing_idle_pose)

        # Do a relative pose movement
        #pose = self.move_group.get_current_pose()
        #pose.posisition.x = pose.posisition.x + 0.1
        success = self.relative_pose_go(0, 0.15, wait=True)

        # Do a series of cartesian movements
        # Go to the middle of the print bed in XY
        pose = list_to_pose_stamped([-0.10, 0.0, 0.2, 0.000000, 1.000000, 0.000000, 0.000000], "world")
        trans_pose = self.transform_pose(pose, "world", "base")
        trans_pose = round_pose_values(trans_pose)
        plan = self.cartesian_go(trans_pose)
        #print(type(plan))
        #print(plan)


if __name__ == "__main__":
    cramped_interface = MoveitInterface(verbose=True)
    cramped_interface.setup(set_planning_time=3)
    cramped_interface.run_demo()

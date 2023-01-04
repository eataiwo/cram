#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
# import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs

from moveit_msgs.msg import DisplayTrajectory
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
from math import pi, atan, tau, dist, fabs, cos, floor
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander.conversions import pose_to_list, list_to_pose, list_to_pose_stamped


# from std_msgs.msg import String

# TODO: Add comprehensive docstring to every method and function.
def round_pose_values(pose, sigfig=6):
    """
    Round pose values
    """
    if type(pose) is PoseStamped:
        pose.pose.position.x = round(pose.pose.position.x, sigfig)
        pose.pose.position.y = round(pose.pose.position.y, sigfig)
        pose.pose.position.z = round(pose.pose.position.z, sigfig)
        pose.pose.orientation.x = round(pose.pose.orientation.x, sigfig)
        pose.pose.orientation.y = round(pose.pose.orientation.y, sigfig)
        pose.pose.orientation.z = round(pose.pose.orientation.z, sigfig)
        pose.pose.orientation.w = round(pose.pose.orientation.w, sigfig)
    elif type(pose) is Pose:
        pose.position.x = round(pose.position.x, sigfig)
        pose.position.y = round(pose.position.y, sigfig)
        pose.position.z = round(pose.position.z, sigfig)
        pose.orientation.x = round(pose.orientation.x, sigfig)
        pose.orientation.y = round(pose.orientation.y, sigfig)
        pose.orientation.z = round(pose.orientation.z, sigfig)
        pose.orientation.w = round(pose.orientation.w, sigfig)

    return pose


class MoveitInterface:
    def __init__(self, group_name="widowx_1", verbose=False,
                 start_state=None):
        """
        Initialises relevant MoveIt things, sets up ROS interfaces, and go to an initial ready pose.
        """
        # For the cartesian interpolation moveit says it needs at least 10 points inbetween waypoints therefore
        # as the repeatability/accuracy is 1mm so that is probably the smallest movement I should really be doing
        # at least for now (07/12/2022)

        # Initialise moveit_commander and a rospy node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_interface", anonymous=False)

        self.eef_step = 0.0001
        # TODO: Waiting to tune this when commissioning the rig
        self.jump_threshold = 0.0
        # TODO: Will revisit the number once I have tested the the workspace reachability and reliability and know what failure rate I can tolerate
        self.planning_attempts = 5

        # Common joint states for a 4DOF arm
        # TODO: Check if this will work if I make them tuples as I do not want these to be overwritten once the program starts running.
        self.print_idle_vertical = [0, 0.46, 1.08913, -0.05522]  # Joint space
        self.print_idle_horizontal = [0, 0.925, 0.81301, -1.7303]  # Joint space
        self.arm_home = (0, -pi / 2, pi / 2, 0)  # Joint space
        self.robot_arm_base_frame = "widowx_1_arm_base_link"
        self.common_poses = {"widowx1_print_home": [],
                             "widowx1_print_idle_vertical": list_to_pose_stamped(
                                 [-0.10, 0.0, 0.2, 0.000000, 1.000000, 0.000000, 0.000000], "base"),
                             "widowx2_print_home": [],
                             }

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

        # TODO: RVIZ maybe already make this publisher so check if there is a conflict or redundancy
        # Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path",
                                                            DisplayTrajectory,
                                                            queue_size=20)

        # Move robot arm to start state
        if start_state is None:
            start_state = self.arm_home  # [0, -pi / 2, pi / 2, 0]
        self.joint_go(start_state, wait=True)
        # TODO: Add roslog into that robot arm (use namespace) has gone home and if it was successful move.

        # Print additional information if verbose set to true.
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
            print("=====================================================")
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

    def pose_for_linear_y_movement(self, pose_target, from_frame="buildplate", pose_start=None):
        """
        Calculating new pose required for linear movement in the y-axis
        """
        # Require a PoseStamped msg. Check is Pose msg, if so convert to PoseStamped msg
        # TODO: Make a better work around for handling a Pose to PoseStamped msg
        # TODO: Make this a private function as the user should not need to use this method directly.
        if type(pose_target) is Pose:
            pose_temp = PoseStamped()
            pose_temp.pose = pose_target
            pose_target = pose_temp
        elif type(pose_target) is PoseStamped:
            pass
        else:
            print("Expected type Pose or PoseStamped")

        try:
            transform = self.tf_buffer.lookup_transform(self.robot_arm_base_frame, from_frame, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        # Transform pose to the reference frame of the robot arm as the rotation will happen around the base
        pose_target = tf2_geometry_msgs.do_transform_pose(pose_target, transform)

        # Work out angle rotated by. Anti-clockwise is a positive theta
        theta = atan(pose_target.pose.position.y / pose_target.pose.position.x)

        # Original quaternion orientation
        q_orig = [pose_target.pose.orientation.x, pose_target.pose.orientation.y, pose_target.pose.orientation.z,
                  pose_target.pose.orientation.w]

        # Rotational quaternion
        q_rot = quaternion_from_euler(0, 0, theta)

        # New quaternion
        q = quaternion_multiply(q_rot, q_orig)

        pose_target.pose.orientation.x = q[0]
        pose_target.pose.orientation.y = q[1]
        pose_target.pose.orientation.z = q[2]
        pose_target.pose.orientation.w = q[3]

        try:
            transform = self.tf_buffer.lookup_transform("base", self.robot_arm_base_frame, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

        pose_target = tf2_geometry_msgs.do_transform_pose(pose_target, transform)

        return pose_target

    def transform_pose(self, pose, from_frame, to_frame):
        """
        Transform a pose from one frame to another
        """
        if type(pose) is PoseStamped:
            pass
        elif type(pose) is Pose:
            pose_temp = PoseStamped()
            pose_temp.pose = pose
            pose = pose_temp
        try:
            transform = self.tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

        transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
        return transformed_pose

    def go(self, wait=True):
        '''
        Execute movement to the current joint value target
        '''
        # TODO: Refactor all code to use this go method
        # TODO: Make this a private function as the user should not need to use this method directly.
        success = False
        attempt = 0
        while not success and attempt < self.planning_attempts:
            success = self.move_group.go(wait)
            attempt += 1
            # TODO: Change print statement to a roslog for ros debugging. Indicate where the statement is coming from
            print(f"=== Movement attempt number: {attempt} ===")
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success

    def joint_go(self, joint_target, wait=True):
        """
        Execute a joint target movement
        """
        self.move_group.set_joint_value_target(joint_target)
        success = self.go(wait)

        return success

    def pose_go(self, pose_target, wait=True):
        """
        Execute a pose goal movement
        """
        # Check for message type
        if type(pose_target) is PoseStamped:
            pose_target.header.stamp = rospy.Time.now()
            # Check is y = 0 if not update quaternion
            # if round(pose_target.pose.position.y, 6) == 0.0:
            #     print("not calculating y pose stamped")
            #     pass
            # elif round(pose_target.pose.position.y, 6) != 0.0:
            #     pose_target.pose = self.pose_for_linear_y_movement(pose_target.pose, "base")
            #     print("calculating y pose stamped")

            pose_target.pose = round_pose_values(pose_target.pose)
            self.move_group.set_pose_target(pose_target.pose)

        elif type(pose_target) is Pose:
            # Check is y = 0 if not update quaternion
            # if round(pose_target.position.y, 6) == 0.0:
            #     print("not calculating y pose not stamped")
            #     pass
            # elif round(pose_target.position.y, 6) != 0.0:
            #     pose_target = self.pose_for_linear_y_movement(pose_target, "base")
            #     print("calculating y pose not stamped")

            pose_target = round_pose_values(pose_target)
            self.move_group.set_pose_target(pose_target)

        success = self.go(wait)
        return success

    def relative_pose_go(self, axis, value, wait=True):  #
        """
        Execute a relative pose movement. Add value to the corresponding axis
        (0..5: X, Y, Z, R, P, Y) and set the new pose as the pose target.
        Note: This does not take a quaternion but roll, pitch, yaw angles.
        """
        self.move_group.shift_pose_target(axis, value)
        success = self.go(wait)
        return success

    def cartesian_go(self, goal, start=None, wait=True):
        """
        Execute a cartesian movement
        """
        # You can plan a Cartesian path directly by specifying a
        # list of waypoints for the end-effector to go through.

        # Initialising an empty list of waypoints
        waypoints = []

        # Setting the start waypoint
        if start is None:
            wpose_start = self.move_group.get_current_pose().pose  # PoseStamped
            waypoints.append(copy.deepcopy(wpose_start))
        elif type(start) is Pose:
            waypoints.append(copy.deepcopy(start))
        elif type(start) is PoseStamped:
            waypoints.append(copy.deepcopy(start.pose))

        # Setting the goal waypoint
        if type(goal) is PoseStamped:
            waypoints.append(copy.deepcopy(goal.pose))
        elif type(goal) is Pose:
            waypoints.append(copy.deepcopy(goal))

        print("=====================================================")
        print(f"=== Waypoints {waypoints} ===")
        print("=====================================================")
        print("\n")

        # We want the Cartesian path to be interpolated at a resolution of 0.1 mm
        # which is why we will specify 0.0001 as the eef_step in Cartesian translation.
        # The jump_threshold will be tuned in the commissioning for now set to 0 to disable checks

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, self.eef_step, self.jump_threshold)  # waypoints to follow  # eef_step # jump_threshold

        # Publish the plan as a trajectory
        # display_trajectory = DisplayTrajectory()
        # display_trajectory.trajectory_start = self.robot.get_current_state()
        # display_trajectory.trajectory.append(plan)
        # self.display_trajectory_publisher.publish(display_trajectory)
        # print("\n")
        # print(f"display trajectory = \n {display_trajectory}")
        # print("\n")

        # Note: We are just planning, not asking move_group to actually move the robot yet:

        # TODO: Add a roslog info message here confirming the movement and the end and start pose if verbose set to true actually do something similar for all movements and reference the namespace too

        success = self.move_group.execute(plan, wait)
        # self.move_group.stop()
        # self.move_group.clear_pose_targets()

        return success

    def go_home(self, pose="widowx1_print_idle_vertical"):
        '''
        Special movement for returning to common poses. For example and pose for an idle state.
        This movement assumes printing could be happening when called therefore the movement will have a specific order
        of waypoints. First a positive relative movement in the z-direction, this is to avoid collision with the
        work piece. Then a cartesian translation to the target xy position. Then a movement in z-direction to the target z-position
        '''
        # TODO: Add a check for the orientation of the end effector to make sure it was in printing mode before doing home movement
        waypoints = []
        successes = []

        # Stop current movement and remove targets
        # TODO: Need to add a way to keep track of what movements have been done versus queued so when a print is interrupted it can resume from where it stopped and not where it has planned up to as clear pose target will erase those trajectories
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Set the start waypoint as the current pose
        wpose = self.move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))

        # Clear the print bed or work piece by 30mm
        wpose.position.z = wpose.position.z + 0.03
        waypoints.append(copy.deepcopy(wpose))
        # success = self.cartesian_go(wpose, wait=True)
        # successes.append(success)

        # Get the target pose from the common pose dictionary a class attribute
        target_pose = self.common_poses[pose]

        # Go to target xy co-ordinates
        wpose.position.x = target_pose.position.x
        wpose.position.y = target_pose.position.y
        waypoints.append(copy.deepcopy(wpose))
        # success = self.cartesian_go(wpose, wait=True)
        # successes.append(success)

        # Finally go to target z-position
        wpose.position.z = target_pose.position.z
        waypoints.append(copy.deepcopy(wpose))
        # success = self.cartesian_go(wpose, wait=True)
        # successes.append(success)

        # if all(successes):
        #     return True
        # else:
        #     return False

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, self.eef_step, self.jump_threshold)  # waypoints to follow  # eef_step # jump_threshold

        # Publish the plan as a trajectory
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        # Note: We are just planning, not asking move_group to actually move the robot yet:

        self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        return fraction

    def run_demo(self):
        """
        Demo of the class functionality.
        """
        vertical_printing_quaternion = (0.000000, 1.000000, 0.000000, 0.000000)
        horizontal_printing_quaternion = (0.000000, 0.7071068, 0.000000, 0.7071068)

        #############################################################################################################
        # Joint space movement
        #############################################################################################################
        # Robot arm should initialise to its default home position but just incase it will be done here.
        # Start at robot arm home position.
        print("=====================================================")
        print(f"================ Joint space test ==================")
        print(f"=== Attempting move to robot home ===")

        success = self.joint_go(self.arm_home, wait=True)

        print("=====================================================")
        print(f"=== Successfully gone to robot home: {success} ===")
        print("\n")
        #############################################################################################################
        # Joint space movement
        #############################################################################################################

        #############################################################################################################
        # Pose movement
        #############################################################################################################
        # Go to vertical printing home position
        print("=====================================================")
        print(f"================ Pose movement test ==================")
        print(f"=== Attempting move to vertical printing idle position ===")

        # In base/world frame
        vertical_printing_idle = [0.100000 - 0.245, 0.000000, 0.100000 + 0.04, 0.000000, 1.000000, 0.000000,
                                  0.000000]  # = [-0.145, 0, 0.14, 0, 1, 0, 0]
        vertical_printing_idle_pose = list_to_pose(vertical_printing_idle)
        success = self.pose_go(vertical_printing_idle_pose)

        print("=====================================================")
        print(f"=== Successfully gone to vertical printing idle position: {success} ===")
        print("\n")
        #############################################################################################################
        # Pose movement
        #############################################################################################################

        #############################################################################################################
        # Relative pose movement
        #############################################################################################################
        # Do a relative pose movement in x-axis
        print("=====================================================")
        print(f"================ Relative pose movement test ==================")
        print(f"=== Attempting a relative move in the x-axis of +0.1m ===")

        success = self.relative_pose_go(0, 0.10, wait=True)
        print(f"=== Successfully executed relative movement in x-axis: {success} ===")
        print("\n")

        # In Z axis
        print(f"=== Attempting a relative move in the z-axis of +0.05m ===")
        success = self.relative_pose_go(2, 0.05, wait=True)
        print(f"=== Successfully executed relative movement in z-axis: {success} ===")
        print("\n")

        # In Y axis - This will not work as the quaternion for the widowx also needs to be calculated which this
        # function lacks
        print(f"=== Attempting a relative move in the y-axis of +0.1m ===")
        success = self.relative_pose_go(1, 0.1, wait=True)
        print(f"=== Successfully executed relative movement in y-axis: {success} ===")
        print("\n")
        #############################################################################################################
        # Relative Pose movement
        #############################################################################################################

        #############################################################################################################
        # Pose movement using transform
        #############################################################################################################
        # Transform required. The final frame required for planner is the base frame, if we want to write commands
        # relative to the buildplate from of reference they need to be the transformed back into the base frame
        # which can then be sent to the planner
        print("=====================================================")
        print(f"================ Pose movement using a transform test ==================")
        print(f"=== Attempting pose movement using the build plate frame of reference ===")

        # Same point as vertical_print_idle in pose_movement defined above but in buildplate frame
        vertical_printing_idle = [0.0005000, 0.150000, 0.100000, 0.000000, 1.000000, 0.000000,
                                  0.000000]
        vertical_printing_idle_pose = list_to_pose(vertical_printing_idle)
        trans_pose = self.transform_pose(vertical_printing_idle_pose, "buildplate", "base")
        trans_pose.pose = round_pose_values(trans_pose.pose)
        success = self.pose_go(trans_pose.pose)
        print(f"=== Successfully gone to vertical printing idle position using the buildplate transform: {success} ===")
        print("\n")

        #############################################################################################################
        # Pose movement using transform
        #############################################################################################################

        #############################################################################################################
        # Pose movement in y-axis using transform
        #############################################################################################################

        # Due to hardware limitations anytime a movement is done the quaternion will also change and this has to be
        # worked out.
        print("=====================================================")
        print(f"================ Pose movement using a transform in y-axis test ==================")
        print(f"=== Attempting pose movement using the widowx_1_arm_base_link frame of reference in y-axis ===")
        pose_target = PoseStamped()
        pose_target.pose = list_to_pose([0.100000, 0.100000, 0.100000, 0.000000, 1.000000, 0.000000, 0.000000])
        if round(pose_target.pose.position.y, 6) != 0.0:
            pose_target = self.pose_for_linear_y_movement(pose_target, "widowx_1_arm_base_link")
        pose_target.pose = round_pose_values(pose_target.pose)
        success = self.pose_go(pose_target.pose)
        print(f"=== Successful pose movement using a transform in y-axis: {success} ===")
        print("\n")

        print(f"=== Attempting pose movement using the buildplate frame of reference in y-axis ===")
        pose_target = PoseStamped()
        pose_target.pose = list_to_pose([0.100000, 0.100000, 0.100000, 0.000000, 1.000000, 0.000000, 0.000000])
        if round(pose_target.pose.position.y, 6) != 15.0:
            pose_target = self.pose_for_linear_y_movement(pose_target, "buildplate")
        pose_target.pose = round_pose_values(pose_target.pose)
        success = self.pose_go(pose_target.pose)
        print(f"=== Successful pose movement using a transform in y-axis: {success} ===")
        print("\n")

        #############################################################################################################
        # Pose movement in y-axis using transform
        #############################################################################################################

        #############################################################################################################
        # Pose movement using transform
        #############################################################################################################
        # Transform required. The final frame required for planner is the base frame, if we want to write commands
        # relative to the buildplate from of reference they need to be the transformed back into the base frame
        # which can then be sent to the planner
        print("=====================================================")
        print(f"================ Pose movement using a transform test ==================")
        print(f"=== Attempting pose movement using the build plate frame of reference ===")

        # Same point as vertical_print_idle in pose_movement defined above but in buildplate frame
        vertical_printing_idle = [0.0005000, 0.150000, 0.100000, 0.000000, 1.000000, 0.000000,
                                  0.000000]
        vertical_printing_idle_pose = list_to_pose(vertical_printing_idle)
        trans_pose = self.transform_pose(vertical_printing_idle_pose, "buildplate", "base")
        trans_pose.pose = round_pose_values(trans_pose.pose)
        success = self.pose_go(trans_pose.pose)
        print(f"=== Successfully gone to vertical printing idle position using the buildplate transform: {success} ===")
        print("\n")

        #############################################################################################################
        # Pose movement using transform
        #############################################################################################################

        #############################################################################################################
        # Cartesian pose movement in y-axis using transform
        #############################################################################################################

        print("=====================================================")
        print(f"================  Cartesian pose movement in y-axis using transform test ==================")
        print(f"=== Attempting pose movement using the buildplate frame of reference ===")

        # Go to the middle of the print bed in XY

        # pose_target = list_to_pose_stamped([0.005000, 0.250000, 0.100000, 0.000000, 1.000000, 0.000000, 0.000000],
        #                                    "buildplate")

        pose_target = list_to_pose_stamped([-0.1, 0.0, 0.2, 0.000000, 1.000000, 0.000000, 0.000000], "base")

        if round(pose_target.pose.position.y, 6) != 0.0:
            pose_target = self.pose_for_linear_y_movement(pose_target, "base")
        pose_target.pose = round_pose_values(pose_target.pose)
        success = self.cartesian_go(pose_target)
        print(f"=== Successful pose movement using a transform in y-axis: {success} ===")
        print("\n")

        # pose_target = list_to_pose_stamped([0.005000, 0.250000, 0.200000, 0.000000, 1.000000, 0.000000, 0.000000],
        #                                    "buildplate")
        #sddfsdf
        pose_target = list_to_pose_stamped([-0.1, 0.1, 0.2, 0.000000, 1.000000, 0.000000, 0.000000], "base")
        if round(pose_target.pose.position.y, 6) != 0.0:
            pose_target = self.pose_for_linear_y_movement(pose_target, "buildplate")
        pose_target.pose = round_pose_values(pose_target.pose)
        success = self.cartesian_go(pose_target)
        print(f"=== Successful pose movement using a transform in y-axis: {success} ===")
        print("\n")

        #############################################################################################################
        # Cartesian pose movement in y-axis using transform
        #############################################################################################################

        # # Do a series of cartesian movements
        # # Do zigzags across the whole printbed between x=0 and x=l/2 where l is the length of the printbed in x
        # x_spacing = 0.15
        # y_spacing = 0.06
        # build_plate_width = 0.3
        # n = (build_plate_width / y_spacing) + 1
        # successes = []
        # for i in range(floor(n)):
        #     pose.pose.position.x = pose.pose.position.x - x_spacing
        #     trans_pose = self.transform_pose(pose, "buildplate", "base")
        #     trans_pose = round_pose_values(trans_pose)
        #     success = self.cartesian_go(trans_pose)
        #     successes.append(success)
        #
        #     pose.pose.position.y = pose.pose.position.y
        #     trans_pose = self.transform_pose(pose, "buildplate", "base")
        #     trans_pose = round_pose_values(trans_pose)
        #     success = self.cartesian_go(trans_pose)
        #     successes
        #
        #     x_spacing = -x_spacing
        # if all(successes):
        #     print(f"=== Successfully executed cartesian zigzag movement over build plate: {success} ===")
        #     print("\n")

        # Go home
        # success = self.go_home("widowx1")
        # print(f"=== Successfully gone to vertical printing home position: {success} ===")
        # print("\n")
        #
        # # Return to robot arm home
        # success = self.joint_go(robot_arm_home, wait=True)
        # print(f"=== Successfully gone to robot home: {success} ===")
        # print("\n")


if __name__ == "__main__":
    cramped_interface = MoveitInterface(verbose=True)
    cramped_interface.setup(set_planning_time=1)
    cramped_interface.run_demo()

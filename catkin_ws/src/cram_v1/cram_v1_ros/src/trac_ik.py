#!/usr/bin/env python

# Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
# Convenience code to wrap TRAC IK

from trac_ik_python.trac_ik_wrap import TRAC_IK
import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import pi
from moveit_commander.conversions import list_to_pose

class IK(object):
    def __init__(self, base_link, tip_link,
                 timeout=5.0, epsilon=1e-4, solve_type="Speed",
                 urdf_string=None):
        """
        Create a TRAC_IK instance and keep track of it.

        :param str base_link: Starting link of the chain.
        :param str tip_link: Last link of the chain.
        :param float timeout: Timeout in seconds for the IK calls.
        :param float epsilon: Error epsilon.
        :param solve_type str: Type of solver, can be:
            Speed (default), Distance, Manipulation1, Manipulation2
        :param urdf_string str: Optional arg, if not given URDF is taken from
            the param server at /robot_description.
        """
        if urdf_string is None:
            urdf_string = rospy.get_param('/robot_description')
        self._urdf_string = urdf_string
        self._timeout = timeout
        self._epsilon = epsilon
        self._solve_type = solve_type
        self.base_link = base_link
        self.tip_link = tip_link
        self._ik_solver = TRAC_IK(self.base_link,
                                  self.tip_link,
                                  self._urdf_string,
                                  self._timeout,
                                  self._epsilon,
                                  self._solve_type)
        self.number_of_joints = self._ik_solver.getNrOfJointsInChain()
        self.joint_names = self._ik_solver.getJointNamesInChain(
            self._urdf_string)
        self.link_names = self._ik_solver.getLinkNamesInChain()

    def get_ik(self, qinit,
               x, y, z,
               rx, ry, rz, rw,
               bx=1e-5, by=1e-5, bz=1e-5,
               brx=1e-3, bry=1e-3, brz=1e-3):
        """
        Do the IK call.

        :param list of float qinit: Initial status of the joints as seed.
        :param float x: X coordinates in base_frame.
        :param float y: Y coordinates in base_frame.
        :param float z: Z coordinates in base_frame.
        :param float rx: X quaternion coordinate.
        :param float ry: Y quaternion coordinate.
        :param float rz: Z quaternion coordinate.
        :param float rw: W quaternion coordinate.
        :param float bx: X allowed bound.
        :param float by: Y allowed bound.
        :param float bz: Z allowed bound.
        :param float brx: rotation over X allowed bound.
        :param float bry: rotation over Y allowed bound.
        :param float brz: rotation over Z allowed bound.

        :return: joint values or None if no solution found.
        :rtype: tuple of float.
        """
        if len(qinit) != self.number_of_joints:
            raise Exception("qinit has length %i and it should have length %i" % (
                len(qinit), self.number_of_joints))
        solution = self._ik_solver.CartToJnt(qinit,
                                             x, y, z,
                                             rx, ry, rz, rw,
                                             bx, by, bz,
                                             brx, bry, brz)
        if solution:
            return solution
        else:
            return None

    def get_joint_limits(self):
        """
        Return lower bound limits and upper bound limits for all the joints
        in the order of the joint names.
        """
        lb = self._ik_solver.getLowerBoundLimits()
        ub = self._ik_solver.getUpperBoundLimits()
        return lb, ub

    def set_joint_limits(self, lower_bounds, upper_bounds):
        """
        Set joint limits for all the joints.

        :arg list lower_bounds: List of float of the lower bound limits for
            all joints.
        :arg list upper_bounds: List of float of the upper bound limits for
            all joints.
        """
        if len(lower_bounds) != self.number_of_joints:
            raise Exception("lower_bounds array size mismatch, it's size %i, should be %i" % (
                len(lower_bounds),
                self.number_of_joints))

        if len(upper_bounds) != self.number_of_joints:
            raise Exception("upper_bounds array size mismatch, it's size %i, should be %i" % (
                len(upper_bounds),
                self.number_of_joints))
        self._ik_solver.setKDLLimits(lower_bounds, upper_bounds)


if __name__ == "__main__":
    ik_solver = IK(base_link="widowx_1_arm_base_link", tip_link="widowx_1_wrist_1_link")
    print(ik_solver.joint_names)
    lb, ub = ik_solver.get_joint_limits()
    print(lb, ub)
    seed = [-0.02914563496982718, -1.547786614976612, 1.5247769031583274,
            -0.018407769454627694]  # Current joint values   # [0.245001, -2e-06, 0.176511, -0.011142, 0.691669, 0.011631, 0.722035]  # Current values of pose

    target_pose = list_to_pose([0.100000, 0.00000111, 0.10000111, 0.000000, 1.000000, 0.000000,
                                0.000000])


    sol = ik_solver.get_ik(seed, target_pose.position.x, target_pose.position.y,
                           target_pose.position.z, target_pose.orientation.x,
                           target_pose.orientation.y, target_pose.orientation.z,
                           target_pose.orientation.w)

    # q = quaternion_from_euler(-pi, 0, -pi + pi/4)
    # sol = ik_solver.get_ik(seed, 0.100000, 0.100000, 0.100000, q[0], q[1], q[2],
    #                        q[3])

    # sol = ik_solver.get_ik(seed, 0.100000, 0.00000, 0.100000, 0, 1, 0, 0)

    print("\n")
    print("=====================================================")
    print(sol)
    print("=====================================================")
    print("\n")

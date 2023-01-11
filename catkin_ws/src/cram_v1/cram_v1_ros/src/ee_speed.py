#!/usr/bin/env python

import rospy
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionFKRequest
from moveit_msgs.srv import GetPositionFKResponse
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped
from std_msgs.msg import Float64
import math

from get_fk import GetFK

def get_vel_from_pose(f_plus, f_minus):

    h = f_plus.header.stamp - f_minus.header.stamp
    h = h.to_sec()
    velx = (f_plus.pose.position.x - f_minus.pose.position.x) / 2 * h
    vely = (f_plus.pose.position.y - f_minus.pose.position.y) / 2 * h
    velz = (f_plus.pose.position.z - f_minus.pose.position.z) / 2 * h

    return [velx, vely, velz]

def vel_2_twist(vel, vel_header):
    twist = TwistStamped()
    twist.header = header
    twist.twist.linear.x = velocity[0]
    twist.twist.linear.y = velocity[1]
    twist.twist.linear.z = velocity[2]
    return twist

def magnitude(vel):
    speed = math.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2 )
    return speed

if __name__ == '__main__':
    rospy.init_node('widowx_1_hotend_velocity')
    rospy.loginfo("Querying for FK")
    gfk = GetFK('widowx_1_hotend', 'base')
    ee_position_publisher = rospy.Publisher("/widowx1/measured_end_effector_pose",
                                            PoseStamped,
                                            queue_size=20)
    ee_velocity_publisher = rospy.Publisher("/widowx1/measured_end_effector_velocity",
                                            TwistStamped,
                                            queue_size=20)
    ee_speed_publisher = rospy.Publisher("/widowx1/measured_end_effector_speed",
                                            Float64,
                                            queue_size=20)

    rate = rospy.Rate(1)
    #pose_t0 = PoseStamped()
    #pose_t0.header.time = rospy.Time.now()

    pose=[]
    for i in range(3):
        resp = gfk.get_current_fk()
        pose.append(resp.pose_stamped[0])
        rospy.sleep(0.1)
    pose_t0 = pose[0]
    pose_t1 = pose[1]
    pose_t2 = pose[2]

    while not rospy.is_shutdown():
        resp = gfk.get_current_fk()
        pose_t2 = resp.pose_stamped[0]
        header = pose_t1.header
        velocity = get_vel_from_pose(pose_t2, pose_t0)
        speed = magnitude(velocity)
        velocity = vel_2_twist(velocity, header)
        print(f"Speed is {speed*1000}mm/s")

        ee_position_publisher.publish(pose_t2)
        ee_velocity_publisher.publish(velocity)
        #ee_speed_publisher.publish(speed)

        pose_t0 = pose_t1
        pose_t1 = pose_t2

        rate.sleep()






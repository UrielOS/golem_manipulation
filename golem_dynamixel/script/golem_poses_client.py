#!/usr/bin/env python

# Author:   Uriel Ortiz
# E-Mail:   euriel.ortiz@gmail.com
# Date:     january/2016

# This node loads default golem poses indicated in 
# meta_controller_motorgroups.yaml, so golem can use them
# through ROS actions.

import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


class MotorGroup:
    def __init__(self, group_name):
        self.name = group_name
        self.jta = actionlib.SimpleActionClient(
                '/'+self.name+'_controller/follow_joint_trajectory',
                FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action...')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')

# Get motor names and positions from parameter server.
        self.motors = rospy.get_param(self.name + '_controller/motors')
        self.poses = rospy.get_param(self.name + '_controller/poses')

    def move_joint(self, angles):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.motors
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(2)
        goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(goal)


def main():
    # r_arm = MotorGroup('right_arm')
    # l_arm = MotorGroup('left_arm')
    r_gripper = MotorGroup('right_gripper')
#    l_gripper = MotorGroup('left_gripper')

#    print "Moving arms..."
#    r_arm.move_joint(r_arm.poses['home'])
#    l_arm.move_joint(l_arm.poses['home'])
#    print "Arm moves have finished!"
#    print r_gripper.motors
    r_gripper.move_joint(r_gripper.poses['open'])
#    l_gripper.move_joint(l_gripper.poses['open'])


if __name__ == '__main__':

    rospy.init_node('joint_poses_node')
    main()

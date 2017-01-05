#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import std_srvs.srv
import golem_kinematics.srv as gk_srv


class MotorGroup:
    def __init__(self, group_name):
        self.name = group_name
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group.set_planner_id('RRTkConfigDefault')

    def init_service(self):
        rospy.Service('/golem_kinematics/' + self.name + '/set_joint_state', gk_srv.JointState, self.set_joint_state)

    def set_joint_state(self, msg):
        resp = gk_srv.JointStateResponse()
        angles = msg.joint_state.position
        self.move_group.clear_pose_targets()
        self.move_group.set_start_state_to_current_state()
        position = list(angles)
        self.move_group.set_joint_value_target(position)
        print 'New goal position for ' + self.name + ':', position
        self.move_group.go()
        # print self.move_group.get_current_joint_values()
        print
        return resp


class Gripper(MotorGroup):
    def __init__(self, group_name):
        MotorGroup.__init__(self, group_name)
        self.state = 'closed'

    def init_service(self):
        rospy.Service('/golem_kinematics/' + self.name + '/change_state', std_srvs.srv.Empty, self.change_state)

    def change_state(self, msg):
        closed = [0.0, 1.5]
        opened = [1.5, 0.0]

        if self.state == 'closed':
            self.move_group.set_joint_value_target(opened)
            print 'Opening gripper: ', self.name, '\n'
            self.move_group.go()
            self.state = 'opened'

        else:
            self.move_group.set_joint_value_target(closed)
            print 'Closing gripper: ', self.name, '\n'
            self.move_group.go()
            self.state = 'closed'

        resp = std_srvs.srv.EmptyResponse()
        return resp


def main():
    print "============ STARTING MOVEGROUP NODE ============="

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_node_milochomil')
    robot = moveit_commander.RobotCommander()

    right_arm = MotorGroup('right_arm')
    right_arm.init_service()

    r_eef = Gripper('right_eef')
    r_eef.init_service()

    # Go to manipulation pose...

    print "============= MOVEGROUP NODE STARTED! =============\n"
    print "Active robot groups:", robot.get_group_names()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print "============= MOVEGROUP NODE ENDED! =============\n"

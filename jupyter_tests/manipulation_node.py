#!/usr/bin/env python

import sys
import rospy as rp
import golem_right_arm_kinematics as right_gk
import golem_right_arm_kinematics as left_gk
import golem_kinematics.srv as gk_srv
import moveit_commander
import std_srvs.srv


class MotorGroup:
    def __init__(self, group_name):
        self.name = group_name
        self.ns = '/golem_kinematics/'
        self.planning_attempts = 10
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group.set_planner_id('RRTConnectkConfigDefault')
        self.move_group.set_workspace([0.1, -1.1, 0.35, 1.3, 0.5, 1.65])  # [minX, minY, minZ, maxX, maxY, maxZ]
        self.move_group.set_planning_time(3.5)
        self.move_group.set_num_planning_attempts(10)
        self.move_group.allow_looking(True)
        rp.Service(self.ns + self.name + '/set_joint_state', gk_srv.JointState, self.joint_state_service)

    def move_to_joint_state(self, angles):
        # arg 'position' must be a list
        self.move_group.clear_pose_targets()
        self.move_group.set_start_state_to_current_state()
        plan = self.compute_plan(angles)
        # print 'New joint state for ' + self.name + ':', angles
        # print 'Moving motors...\n'
        result = self.move_group.execute(plan)
        return result

    def joint_state_service(self, msg):
        resp = gk_srv.JointStateResponse()
        angles = msg.joint_state.position
        position = list(angles)
        resp.result = self.move_to_joint_state(position)
        return resp

    def compute_plan(self, pose):
        self.move_group.set_joint_value_target(pose)
        my_plan = []
        for i in range(self.planning_attempts):
            my_plan.append(self.move_group.plan())
            if my_plan[i].joint_trajectory.points:
                print 'Plan solution found in', i + 1, 'attempts!'
                return my_plan[i]
        print 'No solution found in', self.planning_attempts, 'attempts :('
        return []


class Arm(MotorGroup):
    def __init__(self, group_name):
        MotorGroup.__init__(self, group_name)

        rp.Service(self.ns + self.name + '/compute_fk', gk_srv.ForwardKinematics, self.fk_service)
        rp.Service(self.ns + self.name + '/compute_ik', gk_srv.InverseKinematics, self.ik_service)
        rp.Service(self.ns + self.name + '/move_to_pose', gk_srv.MoveToPose, self.move2pose_service)
        # Pick service
        # Place service

    def forward_kinematics(self, th1, th2, th3, th4, th5, th6):
        if self.name == 'right_arm':
            fk = right_gk.forward_kinematics(th1, th2, th3, th4, th5, th6)

        elif self.name == 'left_arm':
            fk = left_gk.forward_kinematics(th1, th2, th3, th4, th5, th6)

        else:
            print 'Group', self.name, 'has no kinematics defined!'
            return []

        return fk

    def fk_service(self, req):
        response = gk_srv.ForwardKinematicsResponse()

        t1 = req.joint_state.position[0]
        t2 = req.joint_state.position[1]
        t3 = req.joint_state.position[2]
        t4 = req.joint_state.position[3]
        t5 = req.joint_state.position[4]
        t6 = req.joint_state.position[5]

        try:
            fk = self.forward_kinematics(t1, t2, t3, t4, t5, t6)

            response.position.x = fk[0]
            response.position.y = fk[1]
            response.position.z = fk[2]
            response.orientation.x = fk[3]
            response.orientation.y = fk[4]
            response.orientation.z = fk[5]
            response.error = False

        except:
            response.error = True

        return response

    def inverse_kinematics(self, x, y, z, yaw=0.0, pitch=0.0, roll=0.0, only_pos=True):
        if self.name == 'right_arm':
            if not only_pos:
                ik = right_gk.inverse_kinematics(x, y, z, yaw, pitch, roll)
            else:
                ik = right_gk.ik_position(x, y, z)

        elif self.name == 'left_arm':
            if not only_pos:
                ik = left_gk.inverse_kinematics(x, y, z, yaw, pitch, roll)
            else:
                ik = left_gk.ik_position(x, y, z)

        else:
            print 'Group', self.name, 'has no kinematics defined!'
            return []

        return ik

    def ik_service(self, req):
        response = gk_srv.InverseKinematicsResponse()

        only_pos = req.only_pos
        xd = req.position.x
        yd = req.position.y
        zd = req.position.z
        yawd = req.orientation.x
        pitchd = req.orientation.y
        rolld = req.orientation.z

        try:
            ik = self.inverse_kinematics(xd, yd, zd, yawd, pitchd, rolld, only_pos)

            ##### Select solution
            if ik:
                ik_sol = ik[0]
            else:
                ik_sol = []
            #####

            response.joint_state.position = ik_sol
            response.error = False

        except:
            response.error = True

        return response

    def move_to_pose(self, x, y, z, yaw=0.0, pitch=0.0, roll=0.0, only_pos=True):
        ik = self.inverse_kinematics(x, y, z, yaw, pitch, roll, only_pos)
        if ik:
            joint_values = ik[0]
            position = list(joint_values)
            print position
            result = self.move_to_joint_state(position)
            print result
        else:
            print 'No IK solution found'
            result = False
        return result

    def move2pose_service(self, req):
        response = gk_srv.MoveToPoseResponse()

        only_pos = req.only_pos
        xd = req.position.x
        yd = req.position.y
        zd = req.position.z
        yawd = req.orientation.x
        pitchd = req.orientation.y
        rolld = req.orientation.z

        response.result = self.move_to_pose(xd, yd, zd, yawd, pitchd, rolld, only_pos)

        return response


class Gripper(MotorGroup):
    def __init__(self, group_name):
        MotorGroup.__init__(self, group_name)
        self.state_flag = True
        self.opened_state = [1.3, 0.2]
        self.closed_state = [0.0, 1.5]
        rp.Service(self.ns + self.name + '/activate_gripper', std_srvs.srv.Empty, self.gripper_service)

    def open_gripper(self):
        self.move_to_joint_state(self.opened_state)
        print 'Opening gripper: ', self.name, '\n'

    def close_gripper(self):
        self.move_to_joint_state(self.closed_state)
        print 'Closing gripper: ', self.name, '\n'

    def gripper_service(self, msg):
        if self.state_flag:
            self.open_gripper()
        else:
            self.close_gripper()
        self.state_flag = not self.state_flag
        resp = std_srvs.srv.EmptyResponse()
        return resp


def main():
    print "====================================================="
    print "============ STARTING MANIPULATION NODE ============="
    print "====================================================="
    rp.init_node('manipulation_node')

    print "\n==== Starting moveit_commander\n"
    moveit_commander.roscpp_initialize(sys.argv)

    print "\n==== Starting active motor groups\n"
    right_arm = Arm('right_arm')
    r_eef = Gripper('right_eef')
    left_arm = Arm('left_arm')
    l_eef = Gripper('left_eef')

    print "\n==== Moving robot to manipulation state\n"
    right_arm.move_to_joint_state([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    r_eef.open_gripper()
    left_arm.move_to_joint_state([0.0, 0.0, 0.0, 0.0, 0.0])
    l_eef.open_gripper()

    print "===== Manipulation node ready!"
    rp.spin()


if __name__ == '__main__':
    try:
        main()
        moveit_commander.os._exit(0)
    except rp.ROSInterruptException:
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
        print 'ROS was interrupted!'
        pass

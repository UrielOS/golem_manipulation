#!/usr/bin/env python

import sys
import rospy as rp
import manipulation_node
import moveit_commander


test = 2

rp.init_node('pruebas_mani_node_milochomil4')
moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

right_arm = manipulation_node.Arm('right_arm')
right_arm.add_end_effector('right_eef')
left_arm = manipulation_node.Arm('left_arm')
left_arm.add_end_effector('left_eef')

right_arm.move_to_joint_state([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
right_arm.eef.open_gripper()
left_arm.move_to_joint_state([0.0, 0.0, 0.0, 0.7, 0.0])
left_arm.eef.close_gripper()

if test == 1:
    right_arm.pick((0.98, -0.2, 1.0), 'coke_04', 'r_wrist', ['r_gripper_m15', 'r_gripper_m17'])
    right_arm.place((0.6, -0.75, 1.03), 'coke_04')
    rp.sleep(20)
    right_arm.move_to_joint_state(right_arm.home)

    right_arm.pick((0.98, -0.2, 1.3), 'coke_02', 'r_wrist', ['r_gripper_m15', 'r_gripper_m17'])
    right_arm.move_to_joint_state(right_arm.home)
    right_arm.place((0.6, -0.75, 1.32), 'coke_02')
    rp.sleep(20)
    right_arm.move_to_joint_state(right_arm.home)

    right_arm.pick((0.98, 0.2, 1.32), 'coke_01', 'r_wrist', ['r_gripper_m15', 'r_gripper_m17'])
    right_arm.place((1.0, -0.2, 0.97), 'coke_01')
    rp.sleep(20)
    right_arm.move_to_joint_state(right_arm.home)

    right_arm.pick((0.98, 0.2, 1.0), 'coke_03', 'r_wrist', ['r_gripper_m15', 'r_gripper_m17'])
    right_arm.place([0.98, -0.25, 1.4], 'coke_03')
    rp.sleep(20)
    right_arm.move_to_joint_state(right_arm.home)

elif test == 2:
    print "\nAttempting to grab coke_04"
    right_arm.pick((0.98, -0.25, 1.0), 'coke_04', 'r_wrist', ['r_gripper_m15', 'r_gripper_m17'])
    right_arm.place((0.6, -0.75, 1.03), 'coke_04')
    rp.sleep(14)
    print "\nMoving to home pose"
    right_arm.move_to_joint_state(right_arm.home)
    rp.sleep(2)

    print "\nAttempting to grab coke_02"
    right_arm.pick((0.97, -0.25, 1.35), 'coke_02', 'r_wrist', ['r_gripper_m15', 'r_gripper_m17'])
    right_arm.place((1.0, -0.25, 1.01), 'coke_02')
    rp.sleep(20)
    print "\nMoving to home pose"
    right_arm.move_to_joint_state(right_arm.home)
    rp.sleep(2)

    print "\nAttempting to grab coke_03"
    right_arm.pick((0.97, 0.15, 1.0), 'coke_03', 'r_wrist', ['r_gripper_m15', 'r_gripper_m17'])
    right_arm.place([0.98, -0.25, 1.4], 'coke_03')
    rp.sleep(35)
    print "\nMoving to home pose"
    right_arm.move_to_joint_state(right_arm.home)
    rp.sleep(2)

    print "\nAttempting to grab coke_01"
    right_arm.pick((0.98, 0.15, 1.35), 'coke_01', 'r_wrist', ['r_gripper_m15', 'r_gripper_m17'])
    right_arm.place((0.6, -0.75, 1.4), 'coke_01')
    rp.sleep(13)
    print "\nMoving to home pose"
    right_arm.move_to_joint_state(right_arm.home)
    rp.sleep(2)

elif test == 3:
    right_arm.move_to_pose(0.98, -0.2, 1.0)
    right_arm.eef.close_gripper()
    right_arm.move_to_joint_state(right_arm.home)

elif test == 4:
    pass


moveit_commander.roscpp_shutdown()
moveit_commander.os._exit(0)

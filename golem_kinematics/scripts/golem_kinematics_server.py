#!/usr/bin/env python

import rospy as rp
import golem_right_arm_kinematics as right_gk
import golem_right_arm_kinematics as left_gk
import golem_kinematics.srv as gk_srv
# import sensor_msgs.msg
# import geometry_msgs.msg
from tf_conversions import transformations


def fk_callback(req):
    resp = gk_srv.ForwardKinematicsResponse()

    # joints = dict(zip(req.joint_state.name, req.joint_state.position))

    # To do:
    #    - Obtain joint names from group_motor

    if req.motor_group == 'right_arm':
        th1 = req.joint_state.position[0]
        th2 = req.joint_state.position[1]
        th3 = req.joint_state.position[2]
        th4 = req.joint_state.position[3]
        th5 = req.joint_state.position[4]
        th6 = req.joint_state.position[5]

    elif req.motor_group == 'left_arm':
        th1 = req.joint_state.position[0]
        th2 = req.joint_state.position[1]
        th3 = req.joint_state.position[2]
        th4 = req.joint_state.position[3]
        th5 = req.joint_state.position[4]
        th6 = req.joint_state.position[5]

    else:
        print 'ERROR: motor_group', req.motor_group, 'is invalid.'
        resp.error = True
        return resp

    fk = left_gk.forward_kinematics(th1, th2, th3, th4, th5, th6)

    quatx, quaty, quatz, quatw = transformations.quaternion_from_euler(fk[4], fk[5], fk[3], axes='szyx')

    resp.pose_stamped.pose.position.x = fk[0]
    resp.pose_stamped.pose.position.y = fk[1]
    resp.pose_stamped.pose.position.z = fk[2]
    resp.pose_stamped.pose.orientation.x = quatx
    resp.pose_stamped.pose.orientation.y = quaty
    resp.pose_stamped.pose.orientation.z = quatz
    resp.pose_stamped.pose.orientation.w = quatw
    resp.error = False

    return resp


def ik_callback(req):
    resp = gk_srv.InverseKinematicsResponse()
    
    xd = req.pose_stamped.pose.position.x
    yd = req.pose_stamped.pose.position.y
    zd = req.pose_stamped.pose.position.z
    quatx = req.pose_stamped.pose.orientation.x
    quaty = req.pose_stamped.pose.orientation.y
    quatz = req.pose_stamped.pose.orientation.z
    quatw = req.pose_stamped.pose.orientation.w
    
    pitchd, yawd, rolld = transformations.euler_from_quaternion((quatx, quaty, quatz, quatw), axes='szyx')
    
    if req.motor_group == 'right_arm':
        resp.joint_state.name = ('m0', 'm10', 'm11', 'm12', 'm13', 'm14')
        ik = right_gk.inverse_kinematics(xd, yd, zd, yawd, pitchd, rolld)

    elif req.motor_group == 'left_arm':
        resp.joint_state.name = ('m0', 'm20', 'm21', 'm22', 'm23', 'm24')
        ik = left_gk.inverse_kinematics(xd, yd, zd, yawd, pitchd, rolld)

    else:
        print 'ERROR: motor_group', req.motor_group, 'is invalid.'
        resp.error = True
        return resp
    
    ##### Select solution
    if ik:
        ik_sol = ik[0]
    else:
        ik_sol = []
    #####

    resp.joint_state.position = ik_sol

    resp.error = False
    
    return resp


def ik_position_callback(req):
    resp = gk_srv.InverseKinematicsResponse()

    xd = req.pose_stamped.pose.position.x
    yd = req.pose_stamped.pose.position.y
    zd = req.pose_stamped.pose.position.z

    if req.motor_group == 'right_arm':
        resp.joint_state.name = ('m0', 'm10', 'm11', 'm12', 'm13', 'm14')
        ik = right_gk.ik_position(xd, yd, zd)

    elif req.motor_group == 'left_arm':
        resp.joint_state.name = ('m0', 'm20', 'm21', 'm22', 'm23', 'm24')
        ik = left_gk.ik_position(xd, yd, zd)

    else:
        print 'ERROR: motor_group', req.motor_group, 'is invalid.'
        resp.error = True
        return resp

    if ik:
        ik_sol = ik[0]
    else:
        ik_sol = []

    resp.joint_state.position = ik_sol

    resp.error = False

    return resp


def golem_kinematics_server():
    rp.init_node('golem_kinematics')
    print 'GOLEM KINEMATICS NODE STARTED!'
    rp.Service('golem_kinematics/right_arm/compute_fk', gk_srv.ForwardKinematics, fk_callback)
    rp.Service('golem_kinematics/right_arm/compute_ik', gk_srv.InverseKinematics, ik_callback)
    rp.Service('golem_kinematics/right_arm/compute_ik_only_position', gk_srv.InverseKinematics, ik_position_callback)

    rp.Service('golem_kinematics/left_arm/compute_fk', gk_srv.ForwardKinematics, fk_callback)
    rp.Service('golem_kinematics/left_arm/compute_ik', gk_srv.InverseKinematics, ik_callback)
    rp.Service('golem_kinematics/left_arm/compute_ik_only_position', gk_srv.InverseKinematics, ik_position_callback)

    print 'CUSTOM KINEMATICS SOLVERS READY!'
    rp.spin()


if __name__ == '__main__':
    try:
        golem_kinematics_server()
    except rp.ROSInterruptException:
        print 'ROS was interrupted!'

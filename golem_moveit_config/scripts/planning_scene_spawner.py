#!/usr/bin/env python

import rospy as rp
import moveit_commander
import geometry_msgs.msg
import tf.transformations
import math

global scene
global robot

def add_bookshelf(name, x, y, rotation):
    pos_x = x
    pos_y = y
    roll = 0.0
    pitch = 0.0
    yaw = rotation
    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    back_pose = geometry_msgs.msg.PoseStamped()
    left_pose = geometry_msgs.msg.PoseStamped()
    right_pose = geometry_msgs.msg.PoseStamped()
    bottom_pose = geometry_msgs.msg.PoseStamped()
    top_pose = geometry_msgs.msg.PoseStamped()
    shelf1_pose = geometry_msgs.msg.PoseStamped()
    shelf2_pose = geometry_msgs.msg.PoseStamped()

    back_pose.header.frame_id = robot.get_planning_frame()
    back_pose.pose.position.x = -0.2*math.sin(yaw) + pos_x
    back_pose.pose.position.y = -0.2 + 0.2*math.cos(yaw) + pos_y
    back_pose.pose.position.z = 0.6
    back_pose.pose.orientation.x = quat[0]
    back_pose.pose.orientation.y = quat[1]
    back_pose.pose.orientation.z = quat[2]
    back_pose.pose.orientation.w = quat[3]
    back_size = (0.9, 0.01, 1.2)

    left_pose.header.frame_id = robot.get_planning_frame()
    left_pose.pose.position.x = 0.45*math.cos(yaw) + pos_x
    left_pose.pose.position.y = -0.195 + 0.45*math.sin(yaw) + pos_y
    left_pose.pose.position.z = 0.6
    left_pose.pose.orientation.x = quat[0]
    left_pose.pose.orientation.y = quat[1]
    left_pose.pose.orientation.z = quat[2]
    left_pose.pose.orientation.w = quat[3]
    left_size = (0.02, 0.4, 1.2)

    right_pose.header.frame_id = robot.get_planning_frame()
    right_pose.pose.position.x = -0.45*math.cos(yaw) + pos_x
    right_pose.pose.position.y = -0.195 - 0.45*math.sin(yaw) + pos_y
    right_pose.pose.position.z = 0.6
    right_pose.pose.orientation.x = quat[0]
    right_pose.pose.orientation.y = quat[1]
    right_pose.pose.orientation.z = quat[2]
    right_pose.pose.orientation.w = quat[3]
    right_size = (0.02, 0.4, 1.2)

    bottom_pose.header.frame_id = robot.get_planning_frame()
    bottom_pose.pose.position.x = 0.0 + pos_x
    bottom_pose.pose.position.y = -0.195 + pos_y
    bottom_pose.pose.position.z = 0.03
    bottom_pose.pose.orientation.x = quat[0]
    bottom_pose.pose.orientation.y = quat[1]
    bottom_pose.pose.orientation.z = quat[2]
    bottom_pose.pose.orientation.w = quat[3]
    bottom_size = (0.88, 0.4, 0.06)

    top_pose.header.frame_id = robot.get_planning_frame()
    top_pose.pose.position.x = 0.0 + pos_x
    top_pose.pose.position.y = -0.195 + pos_y
    top_pose.pose.position.z = 1.19
    top_pose.pose.orientation.x = quat[0]
    top_pose.pose.orientation.y = quat[1]
    top_pose.pose.orientation.z = quat[2]
    top_pose.pose.orientation.w = quat[3]
    top_size = (0.88, 0.4, 0.02)

    shelf1_pose.header.frame_id = robot.get_planning_frame()
    shelf1_pose.pose.position.x = 0.0 + pos_x
    shelf1_pose.pose.position.y = -0.195 + pos_y
    shelf1_pose.pose.position.z = 0.8
    shelf1_pose.pose.orientation.x = quat[0]
    shelf1_pose.pose.orientation.y = quat[1]
    shelf1_pose.pose.orientation.z = quat[2]
    shelf1_pose.pose.orientation.w = quat[3]
    shelf1_size = (0.88, 0.4, 0.02)

    shelf2_pose.header.frame_id = robot.get_planning_frame()
    shelf2_pose.pose.position.x = 0.0 + pos_x
    shelf2_pose.pose.position.y = -0.195 + pos_y
    shelf2_pose.pose.position.z = 0.43
    shelf2_pose.pose.orientation.x = quat[0]
    shelf2_pose.pose.orientation.y = quat[1]
    shelf2_pose.pose.orientation.z = quat[2]
    shelf2_pose.pose.orientation.w = quat[3]
    shelf2_size = (0.88, 0.4, 0.02)

    scene.add_box(name + 'back', back_pose, back_size)
    scene.add_box(name + 'left', left_pose, left_size)
    scene.add_box(name + 'right', right_pose, right_size)
    scene.add_box(name + 'bottom', bottom_pose, bottom_size)
    scene.add_box(name + 'top', top_pose, top_size)
    scene.add_box(name + 'shelf1', shelf1_pose, shelf1_size)
    scene.add_box(name + 'shelf2', shelf2_pose, shelf2_size)


def add_coke(name, x, y, z):
    coke_pose = geometry_msgs.msg.PoseStamped()
    coke_pose.header.frame_id = robot.get_planning_frame()
    coke_pose.pose.position.x = x
    coke_pose.pose.position.y = y
    coke_pose.pose.position.z = z
    coke_size = (0.14, 0.14, 0.29)

    scene.add_box(name, coke_pose, coke_size)


def main():
    print "======================================================"
    print "============ PLANNING_SCENE_SPAWNER NODE ============="
    print "======================================================"
    rp.wait_for_service('/move_group/planning_scene_monitor/set_parameters')
    rp.init_node('planning_scene_spawner_node')

    global robot
    global scene
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()


    rp.sleep(1)
    print "\n==== Publishing planning scene ====\n"

    print 'Added bookshelf01'
    add_bookshelf('bookshelf01', 1.1, 0.2, -1.5707)
    rp.sleep(1)

    print 'Added bookshelf02'
    add_bookshelf('bookshelf02', 0.54, -0.61, -2.8)
    rp.sleep(1)

    print 'Added coke_01'
    add_coke('coke_01', 1.0, 0.15, 1.36)
    print 'Added coke_02'
    add_coke('coke_02', 1.0, -0.25, 1.36)
    print 'Added coke_03'
    add_coke('coke_03', 1.0, 0.15, 0.965)
    print 'Added coke_04'
    add_coke('coke_04', 1.0, -0.25, 0.965)
    rp.sleep(2)

    print "\n==== Done! ===="
    print "Known objects:"
    object_list = scene.get_known_object_names()
    for object in object_list:
        print '  -', object


if __name__ == '__main__':
    try:
        main()
        moveit_commander.os._exit(0)
    except rp.ROSInterruptException:
        moveit_commander.os._exit(0)
        print 'planning_scene_spawner node ended!'
        pass
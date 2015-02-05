#!/usr/bin/env python
import roslib
roslib.load_manifest('adhenning_project')

import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, \
    roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_pick_py', anonymous=True)

    scene = PlanningSceneInterface()
    robot = RobotCommander()
    arm = robot.arm
    arm.set_start_state_to_current_state()
    arm.set_planner_id("RRTstarkConfigDefault")
    rospy.sleep(1)

    print dir(robot)
    print dir(arm)
    print help(arm.pick)
    print arm.pick

    print "Cleaning the scene"
    scene.remove_world_object("pole")
    scene.remove_world_object("table")
    scene.remove_world_object("part")

    print "Publish a demo scene"
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.7
    p.pose.position.y = -0.4
    p.pose.position.z = 0.85
    p.pose.orientation.w = 1.0
    scene.add_box("pole", p, (0.3, 0.1, 1.0))

    p.pose.position.y = -0.2
    p.pose.position.z = 0.175
    scene.add_box("table", p, (0.5, 1.5, 0.35))

    p.pose.position.x = 0.6
    p.pose.position.y = -0.7
    p.pose.position.z = 0.5
    scene.add_box("part", p, (0.15, 0.1, 0.3))
    rospy.sleep(1)

    print "Pick an object"
    robot.arm.pick("part")

    rospy.spin()
    roscpp_shutdown()

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
    rospy.init_node('pick_cylinder_py', anonymous=True)

    scene = PlanningSceneInterface()
    robot = RobotCommander()
    arm = robot.arm
    arm.set_start_state_to_current_state()
    arm.set_planner_id("RRTstarkConfigDefault")
    arm.set_planning_time(30)
    rospy.sleep(2)

    # print dir(robot)
    # print dir(arm)
    # print help(arm.pick)
    # print arm.pick

    print "Cleaning the previous scene"
    scene.remove_world_object("Floor")
    scene.remove_world_object("Box")

    print "Publishing a demo scene...", robot.get_planning_frame()
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    scene.add_plane("Floor", p)

    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = -0.2
    p.pose.position.y = 0.5
    p.pose.position.z = 0.075/2
    scene.add_box("Box", p, (0.075, 0.075, 0.075))

    # self.setColor("Floor", 0, 0.8, 0, 0.75)
    # self.setColor("Box", 0, 0, 1.00, 1.0)
    # self.sendColors()

    rospy.sleep(1)
    print "...Scene published."

    # print "Pick an object"
    # robot.arm.pick("part")

    rospy.spin()
    roscpp_shutdown()

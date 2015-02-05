#!/usr/bin/env python
import roslib
roslib.load_manifest('adhenning_project')

import sys
import rospy
from moveit_commander import RobotCommander, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import RobotState

if __name__ == '__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_planning_py', anonymous=True)

    robot = RobotCommander()
    rospy.sleep(1)

    print robot.__dict__

    print "Current state:"
    print robot.get_current_state()

    # plan to a random location
    arm = robot.arm
    # arm.set_start_state(RobotState())
    arm.set_start_state_to_current_state()
    arm.set_planner_id("RRTstarkConfigDefault")
    target = arm.get_random_joint_values()
    # arm.set_pose_target(target)

    print "Planning to random joint position: "
    print target
    path = arm.plan(target)
    print "Solution:"
    print path

    if raw_input("Continue [y/n]: ").lower() in ["y", "ye", "yes"]:
        arm.execute(path)

    roscpp_shutdown()

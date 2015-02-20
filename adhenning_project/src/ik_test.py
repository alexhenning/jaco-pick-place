#!/usr/bin/env python
import roslib
roslib.load_manifest('adhenning_project')

import rospy
from std_msgs.msg import Header
from moveit_commander import MoveGroupCommander
from moveit_commander import RobotCommander, PlanningSceneInterface
from moveit_commander import roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

import sys
import random
import cPickle as pickle
from math import sqrt, cos, sin, pi

WORKSPACE = [0, 0, 0, 2, 2, 2]
SAMPLES = 10000


def rand(min, max):
    return min + random.random() * (max - min)


def random_point(workspace):
    return Point(
        x=rand(workspace[0], workspace[3]),
        y=rand(workspace[1], workspace[4]),
        z=rand(workspace[2], workspace[5]),
    )


def random_quaternion():
    u1, u2, u3 = random.random(), random.random(), random.random()
    return Quaternion(
        w=sqrt(1.0-u1)*sin(2.0*pi*u2),
        x=sqrt(1.0-u1)*cos(2.0*pi*u2),
        y=sqrt(u1)*sin(2.0*pi*u3),
        z=sqrt(u1)*cos(2.0*pi*u3),
    )


def get_ik(ik, target, seed):
    response = ik(
        GetPositionIKRequest(
            ik_request=PositionIKRequest(
                group_name="arm",
                pose_stamped=PoseStamped(
                    header=Header(frame_id="root"),
                    pose=target,
                ),
                robot_state=RobotState(joint_state=seed),
                avoid_collisions=True,
            )
        )
    )
    return response


def current_seed(group):
    return JointState(
        name=['jaco_joint_1', 'jaco_joint_2', 'jaco_joint_3',
              'jaco_joint_4', 'jaco_joint_5', 'jaco_joint_6'],
        position=group.get_current_joint_values(),
    )


def collect(seed):
    print "Waiting for service `compute_ik`"
    rospy.wait_for_service('compute_ik')
    ik = rospy.ServiceProxy("compute_ik", GetPositionIK)

    found = []
    while len(found) < SAMPLES:
        print len(found)
        target = Pose(
            position=random_point(WORKSPACE),
            orientation=random_quaternion(),
        )
        response = get_ik(ik, target, seed)
        print target
        print response
        if response.error_code.val == 1:
            found.append((target, response))

    pickle.dump([t for t, _ in found], file("targets.txt", "w"))


def test(seed):
    print "Waiting for service `compute_ik`"
    rospy.wait_for_service('compute_ik')
    ik = rospy.ServiceProxy("compute_ik", GetPositionIK)

    targets = pickle.load(file("targets.txt"))

    tot_score = 0
    successes = 0
    for i, target in enumerate(targets):
        print i
        response = get_ik(ik, target, seed)
        if response.error_code.val != 1:
            print "Failed to find solution"
        else:
            # print response.solution
            current = response.solution.joint_state
            # print seed.position
            # print current

            score = 0
            for name in seed.name:
                start = seed.position[seed.name.index(name)]
                actual = current.position[current.name.index(name)]
                delta = actual - start
                # print "%s-%s = %s" % (actual, start, delta)
                score += abs(delta)
            print score
            tot_score += score
            successes += 1

    print "Average Score: %s, %s successes out of %s trials." % (tot_score/successes, successes, len(targets))

if __name__ == '__main__':
    try:
        roscpp_initialize(sys.argv)
        print sys.argv
        rospy.init_node('adhenning_iktest', anonymous=True)

        # scene = PlanningSceneInterface()
        robot = RobotCommander()
        group = robot.arm
        group.set_planner_id("RRTstarkConfigDefault")
        rospy.sleep(1)
        group.set_start_state_to_current_state()
        # group = MoveGroupCommander("right_arm")

        # print type(group.get_current_joint_values()), group.get_current_joint_values()
        seed = current_seed(group)
        # print seed
        # if "collect" in sys.argv:
        #     collect(seed)
        # else:
        #     test(seed)

        print "Waiting for service `compute_ik`"
        rospy.wait_for_service('compute_ik')
        ik = rospy.ServiceProxy("compute_ik", GetPositionIK)
        # target = Pose(
        #     position=Point(x=-0.3, y=0.5, z=0.4),
        #     orientation=Quaternion(x=0.707107, y=0, z=0, w=0.707107),
        # )
        # -0.299675 0.498836 0.24692 0.999969 -0.0041638 -0.00632961 -0.00191499
        target = Pose(
            position=Point(x=-0.299675, y=0.498836, z=0.24692),
            orientation=Quaternion(x=0.999969, y=-0.0041638, z=-0.00632961, w=-0.00191499),
        )
        print get_ik(ik, target, seed)

        # group.set_joint_value_target(response.solution.joint_state)

        # move to a random target
        # print group.get_workspace()
        # print "Starting"
        # [minX, minY, minZ, maxX, maxY, maxZ]
        # group.set_workspace(WORKSPACE)
        # print "Set workspace"
        # group.set_joint_value_target(Pose(
        #     position=random_point(WORKSPACE),
        #     orientation=Quaternion(w=1)
        # ))
        # group.go()

        # rospy.spin()
    finally:
        roscpp_shutdown()

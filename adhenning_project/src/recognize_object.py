#! /usr/bin/env python

import roslib
roslib.load_manifest('adhenning_project')
import rospy

import actionlib
from std_msgs.msg import Header
from object_recognition_msgs.msg import ObjectRecognitionAction, \
    ObjectRecognitionResult, RecognizedObjectArray, RecognizedObject, \
    ObjectType
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, \
    Pose, Point, Quaternion


class ObjectRecognizer(object):
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            ObjectRecognitionAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._as.start()

    def execute_cb(self, goal):
        # publish info to the console for the user
        rospy.loginfo('%s: %s' % (self._action_name, goal))

        # create messages that are used to publish feedback/result
        objects = []

        o = RecognizedObject(
            header=Header(stamp=rospy.Time.now()),
            # object_recognition_msgs/ObjectType
            # type=ObjectType(key="book", db=""),
            type=ObjectType(key="202b4572b37745aa69ef6f41f00005cc",
                            db="{'type':'CouchDB', 'root':'http://localhost:5984'}"),
            # Confidence that this is the object
            confidence=1,
            # point_clouds
            # bounding_mesh
            # geometry_msgs/PoseWithCovarianceStamped
            pose=PoseWithCovarianceStamped(
                header=Header(stamp=rospy.Time.now()),
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(x=0.20, y=0.75, z=0.25),
                        orientation=Quaternion(),
                    ),
                ),
            ),
        )
        objects.append(o)

        recognized_objects = RecognizedObjectArray(
            header=Header(stamp=rospy.Time.now()),
            objects=objects,
        )
        result = ObjectRecognitionResult(recognized_objects=recognized_objects)
        rospy.loginfo('%s: %s' % (self._action_name, result))
        self._as.set_succeeded(result)
        rospy.loginfo('%s: Succeeded' % self._action_name)

if __name__ == '__main__':
    rospy.init_node('recognize_objects')
    rospy.loginfo("%s starting up..." % rospy.get_name())
    ObjectRecognizer(rospy.get_name())
    rospy.loginfo("%s listening..." % rospy.get_name())
    rospy.spin()

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Tests the grasp generator
*/

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace baxter_pick_place
{

static const double BLOCK_SIZE = 0.075;

class GraspGeneratorTest
{
private:
  // A shared node handle
  ros::NodeHandle nh_;

  // class for publishing stuff to rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // our interface with MoveIt
  boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;

  // which baxter arm are we using
  std::string ee_group_name_;
  std::string planning_group_name_;

public:

  // Constructor
  GraspGeneratorTest(int num_tests)
    : nh_("~")
  {
    nh_.param("ee_group_name", ee_group_name_, std::string("gripper"));
    planning_group_name_ = "arm";
    
    // Create MoveGroup for one of the planning groups
    move_group_.reset(new move_group_interface::MoveGroup(planning_group_name_));
    move_group_->setPlanningTime(30.0);

    ROS_INFO_STREAM_NAMED("test","End Effector: " << ee_group_name_);
    ROS_INFO_STREAM_NAMED("test","Planning Group: " << planning_group_name_);

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("/root"));
    visual_tools_->setLifetime(120.0);
    visual_tools_->setMuted(false);
    visual_tools_->loadMarkerPub();

    geometry_msgs::Pose pose;
    visual_tools_->generateEmptyPose(pose);

    // ---------------------------------------------------------------------------------------------
    // Animate open and closing end effector

    // visual_tools_->loadEEMarker(grasp_data_.ee_group_, planning_group_name_);
    // for (std::size_t i = 0; i < 4; ++i)
    // {
    //   // Test visualization of end effector in OPEN position
    //   grasp_data_.setRobotStatePreGrasp( visual_tools_->getSharedRobotState() );
    //   visual_tools_->loadEEMarker(grasp_data_.ee_group_, planning_group_name_);
    //   visual_tools_->publishEEMarkers(pose, rviz_visual_tools::ORANGE, "test_eef");
    //   ros::Duration(1.0).sleep();

    //   // Test visualization of end effector in CLOSED position
    //   grasp_data_.setRobotStateGrasp( visual_tools_->getSharedRobotState() );
    //   visual_tools_->loadEEMarker(grasp_data_.ee_group_, planning_group_name_);
    //   visual_tools_->publishEEMarkers(pose, rviz_visual_tools::GREEN, "test_eef");
    //   ros::Duration(1.0).sleep();
    // }

    // ---------------------------------------------------------------------------------------------
    // Generate grasps for a bunch of random objects
    geometry_msgs::Pose object_pose;

    // Loop
    int i = 0;
    while(ros::ok())
    {
      ROS_INFO_STREAM_NAMED("test","Adding random object " << i+1 << " of " << num_tests);

      // Remove randomness when we are only running one test
      generateTestObject(i, object_pose);
      
      // Show the block
      // visual_tools_->publishBlock(object_pose, rviz_visual_tools::BLUE, BLOCK_SIZE);
      visual_tools_->publishCollisionFloor(0.0, "Floor", rviz_visual_tools::BLUE);
      std::ostringstream stringStream;
      stringStream << "Block " << i ;
      std::string block_name = stringStream.str();
      visual_tools_->publishCollisionBlock(object_pose, block_name, BLOCK_SIZE);

      std::vector<moveit_msgs::Grasp> possible_grasps;
      possible_grasps.clear();
      
      moveit_msgs::Grasp possible_grasp;
      possible_grasp.id = "My Grasp";
      possible_grasp.grasp_quality = 1.0;

      std::vector<std::string> joint_names;
      joint_names.push_back("jaco_joint_finger_1");
      joint_names.push_back("jaco_joint_finger_2");
      joint_names.push_back("jaco_joint_finger_3");
      joint_names.push_back("jaco_joint_finger_tip_1");
      joint_names.push_back("jaco_joint_finger_tip_2");
      joint_names.push_back("jaco_joint_finger_tip_3");

      trajectory_msgs::JointTrajectory pre_grasp_posture;
      pre_grasp_posture.header.frame_id = "root";
      pre_grasp_posture.header.stamp = ros::Time::now();
      pre_grasp_posture.joint_names = joint_names;
      pre_grasp_posture.points.resize(1);
      pre_grasp_posture.points[0].positions.push_back(0);
      pre_grasp_posture.points[0].positions.push_back(0);
      pre_grasp_posture.points[0].positions.push_back(0);
      pre_grasp_posture.points[0].positions.push_back(0);
      pre_grasp_posture.points[0].positions.push_back(0);
      pre_grasp_posture.points[0].positions.push_back(0);
      pre_grasp_posture.points[0].time_from_start = ros::Duration(4.0);
      possible_grasp.pre_grasp_posture = pre_grasp_posture;

      trajectory_msgs::JointTrajectory grasp_posture;
      grasp_posture.header.frame_id = "root";
      grasp_posture.header.stamp = ros::Time::now();
      grasp_posture.joint_names = joint_names;
      grasp_posture.points.resize(1);
      grasp_posture.points[0].positions.push_back(0.175);
      grasp_posture.points[0].positions.push_back(0.175);
      grasp_posture.points[0].positions.push_back(0.175);
      grasp_posture.points[0].positions.push_back(0.5);
      grasp_posture.points[0].positions.push_back(0.5);
      grasp_posture.points[0].positions.push_back(0.5);
      grasp_posture.points[0].time_from_start = ros::Duration(4.0);
      possible_grasp.grasp_posture = grasp_posture;

      geometry_msgs::PoseStamped grasp_pose_msg;
      grasp_pose_msg.header.stamp = ros::Time::now();
      grasp_pose_msg.header.frame_id = "/root";
      grasp_pose_msg.pose = object_pose;

      grasp_pose_msg.pose.position.z += 0.2;

      // Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitZ());
      // Eigen::AngleAxisd yawAngle(-1.45, Eigen::Vector3d::UnitY());
      // Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitX());
      // Eigen::Quaternion<double> quat = rollAngle * yawAngle * pitchAngle;
      // grasp_pose_msg.pose.orientation.x = quat.x();
      // grasp_pose_msg.pose.orientation.y = quat.y();
      // grasp_pose_msg.pose.orientation.z = quat.z();
      // grasp_pose_msg.pose.orientation.w = quat.w();
      
      grasp_pose_msg.pose.orientation.x = 0;
      grasp_pose_msg.pose.orientation.y = 0;
      grasp_pose_msg.pose.orientation.z = 0;
      grasp_pose_msg.pose.orientation.w = 1;
      possible_grasp.grasp_pose = grasp_pose_msg;

      moveit_msgs::GripperTranslation pre_grasp_approach;
      pre_grasp_approach.direction.header.frame_id = "/root";
      pre_grasp_approach.direction.header.stamp = ros::Time::now();
      pre_grasp_approach.desired_distance = 0.6;
      pre_grasp_approach.min_distance = 0.4;
      pre_grasp_approach.direction.vector.x = 0;
      pre_grasp_approach.direction.vector.y = 0;
      pre_grasp_approach.direction.vector.z = -1; // Approach direction (negative z axis)
      possible_grasp.pre_grasp_approach = pre_grasp_approach;

      moveit_msgs::GripperTranslation post_grasp_retreat;
      post_grasp_retreat.direction.header.frame_id = "/root";
      post_grasp_retreat.direction.header.stamp = ros::Time::now();
      post_grasp_retreat.desired_distance = 0.6;
      post_grasp_retreat.min_distance = 0.4;
      post_grasp_retreat.direction.vector.x = 0;
      post_grasp_retreat.direction.vector.y = 0;
      post_grasp_retreat.direction.vector.z = 1; // Retreat direction (pos z axis)
      possible_grasp.post_grasp_retreat = post_grasp_retreat;
      
      // possible_grasp.post_place_retreat
      possible_grasp.allowed_touch_objects.push_back(block_name);

      possible_grasps.push_back(possible_grasp);
      visual_tools_->loadEEMarker("gripper", "arm");
      std::cout << "Showing grasp: " << block_name << std::endl;
      visual_tools_->publishGrasps(possible_grasps, "jaco_link_hand");
      ros::Duration(2.0).sleep();
      std::cout << "Showing animated grasp: " << block_name << std::endl;
      visual_tools_->publishAnimatedGrasps(possible_grasps, "jaco_link_hand");
      
      move_group_->setSupportSurfaceName("Floor");
      move_group_->setPlannerId("RRTstarkConfigDefault");
      move_group_->setPlanningTime(30);
      std::cout << "Ready to pick: " << block_name << std::endl;
      ros::Duration(0.5).sleep(); 
      std::cout << "Picking: " << block_name << std::endl;
      move_group_->pick(block_name, possible_grasp);
      
      // Test if done
      ++i;
      if( i >= num_tests )
        break;
    }
  }

  void generateTestObject(int i, geometry_msgs::Pose& object_pose)
  {
    // Position
    geometry_msgs::Pose start_object_pose;

    switch (i)
    {
    case 0:
      start_object_pose.position.x = 0.2;
      start_object_pose.position.y = 0.5;
      break;
    case 1:
      start_object_pose.position.x = 0.4;
      start_object_pose.position.y = -0.2;
      break;
    case 2:
      start_object_pose.position.x = 0.4;
      start_object_pose.position.y = -0.2;
      break;
    case 3:
      start_object_pose.position.x = 0.4;
      start_object_pose.position.y = -0.2;
      break;
    default:
      return generateRandomObject(object_pose);
    }

    start_object_pose.position.z =  BLOCK_SIZE/2;

    // Orientation
    double angle = 0; // M_PI / 1.5;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    start_object_pose.orientation.x = quat.x();
    start_object_pose.orientation.y = quat.y();
    start_object_pose.orientation.z = quat.z();
    start_object_pose.orientation.w = quat.w();

    // Choose which object to test
    object_pose = start_object_pose;

    //visual_tools_->publishObject( object_pose, OBJECT_SIZE, true );
  }

  void generateRandomObject(geometry_msgs::Pose& object_pose)
  {
    // Position
    object_pose.position.x = fRand(0.1,0.9); //0.55);
    object_pose.position.y = fRand(-0.28,0.28);
    object_pose.position.z = 0.02;

    // Orientation
    double angle = M_PI * fRand(0.1,1);
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    object_pose.orientation.x = quat.x();
    object_pose.orientation.y = quat.y();
    object_pose.orientation.z = quat.z();
    object_pose.orientation.w = quat.w();
  }

  double fRand(double fMin, double fMax)
  {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
  }

}; // end of class

} // namespace


int main(int argc, char *argv[])
{
  int num_tests = 1;
  ros::init(argc, argv, "grasp_generator_test");

  ROS_INFO_STREAM_NAMED("main","Simple Grasps Test");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Seed random
  srand(ros::Time::now().toSec());

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();

  // Run Tests
  baxter_pick_place::GraspGeneratorTest tester(num_tests);

  // Benchmark time
  double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  ROS_INFO_STREAM_NAMED("","Total time: " << duration);
  //std::cout << duration << "\t" << num_tests << std::endl;

  ros::Duration(1.0).sleep(); // let rviz markers finish publishing

  return 0;
}

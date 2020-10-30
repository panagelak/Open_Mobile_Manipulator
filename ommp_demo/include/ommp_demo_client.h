#ifndef OMMP_CLIENT_H
#define OMMP_CLIENT_H

#include <pwd.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sys/ioctl.h>  // For ioctl, TIOCGWINSZ
#include <unistd.h>     // For STDOUT_FILENO
#include <fstream>

#include <math.h>
#include <vector>

#include <actionlib/client/simple_action_client.h>

#include <integration/AddBox.h>
#include <integration/GetFrameDistance.h>
#include <integration/MoveToJointsAction.h>
#include <integration/MoveToPoseAction.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>

// demo utils
#include "demo_utils.h"

//=========================================================================//
//==================== ACTION AND SERVICE CLIENTS =========================//
//=========================================================================//

static actionlib::SimpleActionClient<integration::MoveToPoseAction> *moveCartesian_arm_ac;
static actionlib::SimpleActionClient<integration::MoveToJointsAction> *moveJoints_gripper_ac;
static actionlib::SimpleActionClient<integration::MoveToJointsAction> *moveJoints_arm_ac;

static actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *move_base_ac;

static ros::ServiceClient *get_frame_distance_clientPrt;
static ros::ServiceClient *add_box_clientPrt;

//=========================================================================//
//=========================================================================//
//=========================================================================//

//=========================================================================//
//==================== TEMPLATE GOALS DEFINITION ==========================//
//=========================================================================//

// Template Goal Cartesian Kuka
static integration::MoveToPoseGoal moveCartesian_arm_template_goal;

// Template Goal Both Grippers arm
static integration::MoveToJointsGoal moveJoints_arm_template_goal;

// Template Goal Joints Kuka
static integration::MoveToJointsGoal moveJoints_gripper_template_goal;

// Template Move Base
static move_base_msgs::MoveBaseGoal move_base_template_goal;

//=========================================================================//
//=========================================================================//
//=========================================================================//

//=========================================================================//
//==================== GOALS DEFINITION ===================================//
//=========================================================================//

//////////Goals Cartesian Arm
static integration::MoveToPoseGoal moveCartesian_arm_test1_goal, moveCartesian_arm_test2_goal;
//////////Goals Joints Arm
static integration::MoveToJointsGoal moveJoints_arm_start_goal, moveJoints_arm_calibrated_goal,
    moveJoints_arm_out_of_view_goal, moveJoints_arm_for_guidance_goal;

//////////Goals Joints Gripper
static integration::MoveToJointsGoal moveJoints_gripper_open_goal, moveJoints_gripper_close_goal,
    moveJoints_gripper_semi_close_goal;

// Services
static integration::GetFrameDistance get_product_position_srv;
static integration::AddBox add_box_test1_srv, remove_box_test1_srv;

// MOVE BASE
static move_base_msgs::MoveBaseGoal move_base_forward_goal, move_base_backward_goal, move_base_start_goal;

//=========================================================================//
//=========================================================================//
//=========================================================================//

// Value Pair (Used for joint Goals)
static integration::PropertyValuePair gripper_temp, arm_temp;

// Empty pose place holder
static geometry_msgs::PoseStamped empty_pose;

// Pose of planning boxes
geometry_msgs::Pose box_pose_test1;

// Speeds and Accelerations of End-Effectors
static float standard_velocity = 1.0f;       // 0.8
static float standard_acceleration = 1.0f;   // 0.8
static float precision_velocity = 0.2f;      // 0.2
static float precision_acceleration = 0.2f;  // 0.2

void createActionClients()
{
  moveCartesian_arm_ac = new actionlib::SimpleActionClient<integration::MoveToPoseAction>("/move_cartesian_arm_"
                                                                                          "handler_server/action",
                                                                                          true);
  moveJoints_arm_ac = new actionlib::SimpleActionClient<integration::MoveToJointsAction>("arm_move_joints_handler_"
                                                                                         "server/action",
                                                                                         true);
  moveJoints_gripper_ac = new actionlib::SimpleActionClient<integration::MoveToJointsAction>("/gripper_"
                                                                                             "move_joints_"
                                                                                             "handler_server/"
                                                                                             "action",
                                                                                             true);
  move_base_ac = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("/move_base", true);
}

void deleteActionClients()
{
  delete moveCartesian_arm_ac;
  delete moveJoints_gripper_ac;
  delete moveJoints_arm_ac;
  delete move_base_ac;
}

void sigIntHandler(int sig)
{
  moveCartesian_arm_ac->cancelAllGoals();
  moveJoints_gripper_ac->cancelAllGoals();
  moveJoints_arm_ac->cancelAllGoals();
  move_base_ac->cancelAllGoals();

  ROS_ERROR("Client was terminated with signal %i. All goals were canceled.", sig);

  deleteActionClients();

  // current_action_publisher.shutdown();
  ros::shutdown();
}

void cancelAllGoals()
{
  moveCartesian_arm_ac->cancelAllGoals();
  moveJoints_gripper_ac->cancelAllGoals();
  moveJoints_arm_ac->cancelAllGoals();
  move_base_ac->cancelAllGoals();
  ROS_ERROR("Canceling all Goals");
}

// CALLBACKS!!!!!!!!!!!!

// void in_zonesCB(const integration::BoolArray::ConstPtr &msg)
//{
//  bool in_zone1 = msg->data[0];
//  bool in_zone2 = msg->data[1];
//  bool in_zone3 = msg->data[2];
//  bool in_zone4 = msg->data[3];
//  if (!suppress_zones && (in_zone1 || in_zone2 || in_zone3 || in_zone4))
//  {
//    cancelAllGoals();
//  }
//}

void setConstantGoalValues()
{
  // moveCartesian_arm template goal
  moveCartesian_arm_template_goal.target_pose = empty_pose;
  moveCartesian_arm_template_goal.target_pose.header.frame_id = "arm_footprint";
  moveCartesian_arm_template_goal.constraint_mode = 0;
  moveCartesian_arm_template_goal.velocity = standard_velocity;
  moveCartesian_arm_template_goal.acceleration = standard_acceleration;
  moveCartesian_arm_template_goal.timeout = 160;

  // moveJoints gripper template goal
  moveJoints_gripper_template_goal.endEffectorVelocity = 1.0;
  moveJoints_gripper_template_goal.endEffectorAcceleration = 1.0;
  moveJoints_gripper_template_goal.timeoutSeconds = 20.0;
  std::vector<std::string> gripper_joints = { "finger1_joint", "finger2_joint" };
  for (std::string name : gripper_joints)
  {
    gripper_temp.name = name;
    moveJoints_gripper_template_goal.joints.push_back(gripper_temp);
  }
  // moveJoints arm template goal
  moveJoints_arm_template_goal.endEffectorVelocity = 1.0;
  moveJoints_arm_template_goal.endEffectorAcceleration = 1.0;
  moveJoints_arm_template_goal.timeoutSeconds = 20.0;
  std::vector<std::string> arm_joints = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                          "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint" };
  for (std::string name : arm_joints)
  {
    arm_temp.name = name;
    moveJoints_arm_template_goal.joints.push_back(arm_temp);
  }

  // MOVE BASE
  move_base_template_goal.target_pose.header.frame_id = "map";
  move_base_template_goal.target_pose.header.stamp = ros::Time::now();
}

void setGoalValues()
{
  //=========================================================================//
  //==================== EXECUTE HUMAN TASK =================================//
  //=========================================================================//
  // human_execute_task_goal = human_template_goal;

  //=========================================================================//
  //==================== ARM JOINT GOALS ===================================//
  //=========================================================================//

  moveJoints_arm_start_goal = moveJoints_arm_template_goal;
  setJointGoalValues(moveJoints_arm_start_goal, { 0.0, -1.2885, 2.1774, 0.0, -1.57, 0.0 });

  moveJoints_arm_calibrated_goal = moveJoints_arm_template_goal;
  setJointGoalValues(moveJoints_arm_calibrated_goal, { 0.0, 0.0, 1.57, 0.0, 0.0, 0.0 });

  moveJoints_arm_for_guidance_goal = moveJoints_arm_template_goal;
  setJointGoalValues(moveJoints_arm_for_guidance_goal, { 0.0, 1.0, 1.0, 0.0, -1.0, 0.0 });

  moveJoints_arm_out_of_view_goal = moveJoints_arm_template_goal;
  setJointGoalValues(moveJoints_arm_out_of_view_goal, { 1.5353, 1.4187, 1.5429, 0.0, -1.57, 0.0 });

  //=========================================================================//
  //==================== GRIPPER JOINT GOALS ===================================//
  //=========================================================================//
  moveJoints_gripper_open_goal = moveJoints_gripper_template_goal;
  setJointGoalValues(moveJoints_gripper_open_goal, { 0.0, 0.0 });

  moveJoints_gripper_semi_close_goal = moveJoints_gripper_template_goal;
  setJointGoalValues(moveJoints_gripper_semi_close_goal, { 0.0125, 0.0125 });

  moveJoints_gripper_open_goal = moveJoints_gripper_template_goal;
  setJointGoalValues(moveJoints_gripper_open_goal, { 0.02, 0.02 });

  //=========================================================================//
  //==================== ARM MoveToPose GOALS ===================================//
  //=========================================================================//
  // Test1 Goal
  moveCartesian_arm_test1_goal = moveCartesian_arm_template_goal;
  moveCartesian_arm_test1_goal.target_pose.header.frame_id = "wrist_3_link";
  moveCartesian_arm_test1_goal.constraint_mode = 0;
  setCartesianGoalValues(moveCartesian_arm_test1_goal, { 0.05, 0.0, -0.05, 0.0, 0.0, 0.0, 1.0 });

  // Test2 Goal
  moveCartesian_arm_test2_goal = moveCartesian_arm_template_goal;
  moveCartesian_arm_test2_goal.target_pose.header.frame_id = "wrist_3_link";
  moveCartesian_arm_test2_goal.constraint_mode = 0;
  setCartesianGoalValues(moveCartesian_arm_test2_goal, { -0.05, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0 });

  //=========================================================================//
  //==================== GET FRAME SERVICE REQUESTS==========================//
  //=========================================================================//

  get_product_position_srv.request.source_frame = "arm_footprint";
  get_product_position_srv.request.target_frame = "object";

  //=========================================================================//
  //==================== ADD BOX SERVICE REQUESTS==========================//
  //=========================================================================/

  // box_pose_test1.orientation.x = -0.574;
  // box_pose_test1.orientation.y = 0.0;
  // box_pose_test1.orientation.z = 0.0;
  // box_pose_test1.orientation.w = 0.819;
  // box_pose_test1.position.x = 0.0;
  // box_pose_test1.position.y = 0.0;
  // box_pose_test1.position.z = 0.7;
  //
  // add_box_test1_srv.request.add = true;
  // add_box_test1_srv.request.planning_frame = "robot_footprint";
  // add_box_test1_srv.request.object_id = "box_test1";
  // add_box_test1_srv.request.size_x = 2.8;
  // add_box_test1_srv.request.size_y = 0.05;
  // add_box_test1_srv.request.size_z = 1.0;
  // add_box_test1_srv.request.object_pose = box_pose_test1;
  //
  // remove_box_test1_srv = add_box_test1_srv;
  // remove_box_test1_srv.request.add = false;

  //=========================================================================//
  //==================== MOVE BASE ==========================================//
  //=========================================================================//
  move_base_forward_goal = move_base_template_goal;
  move_base_forward_goal.target_pose.pose.position.x = 1.5;
  move_base_forward_goal.target_pose.pose.position.y = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 1.57);
  move_base_forward_goal.target_pose.pose.orientation.x = q[0];
  move_base_forward_goal.target_pose.pose.orientation.y = q[1];
  move_base_forward_goal.target_pose.pose.orientation.z = q[2];
  move_base_forward_goal.target_pose.pose.orientation.w = q[3];

  move_base_backward_goal = move_base_template_goal;
  move_base_backward_goal.target_pose.pose.position.x = -1.5;
  move_base_backward_goal.target_pose.pose.position.y = 0.0;

  q.setRPY(0, 0, -1.57);
  move_base_backward_goal.target_pose.pose.orientation.x = q[0];
  move_base_backward_goal.target_pose.pose.orientation.y = q[1];
  move_base_backward_goal.target_pose.pose.orientation.z = q[2];
  move_base_backward_goal.target_pose.pose.orientation.w = q[3];

  move_base_start_goal = move_base_template_goal;
  move_base_start_goal.target_pose.pose.position.x = 0.0;
  move_base_start_goal.target_pose.pose.position.y = 0.0;

  q.setRPY(0, 0, 0);
  move_base_start_goal.target_pose.pose.orientation.x = q[0];
  move_base_start_goal.target_pose.pose.orientation.y = q[1];
  move_base_start_goal.target_pose.pose.orientation.z = q[2];
  move_base_start_goal.target_pose.pose.orientation.w = q[3];
}

#endif  // AERNNOVA_CLIENT_H

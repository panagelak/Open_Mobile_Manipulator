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
#include <integration/AttachObjectAction.h>
#include <integration/GetFrameDistance.h>
#include <integration/MoveModelAction.h>
#include <integration/MoveToJointsAction.h>
#include <integration/MoveToPoseAction.h>
#include <integration/SpawnModels.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

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

static actionlib::SimpleActionClient<integration::MoveToPoseAction> *moveToPose_arm_ac;
static actionlib::SimpleActionClient<integration::MoveToJointsAction> *moveJoints_gripper_ac;
static actionlib::SimpleActionClient<integration::MoveToJointsAction> *moveJoints_arm_ac;
static actionlib::SimpleActionClient<integration::MoveModelAction> *model_move_ac;
static actionlib::SimpleActionClient<integration::AttachObjectAction> *attach_object_ac;
static actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *move_base_ac;

static ros::ServiceClient *get_frame_distance_clientPrt;
static ros::ServiceClient *add_box_clientPrt;
static ros::ServiceClient *spawn_models_clientPrt;

//=========================================================================//
//=========================================================================//
//=========================================================================//

//=========================================================================//
//==================== TEMPLATE GOALS DEFINITION ==========================//
//=========================================================================//

// Template Goal Cartesian Kuka
static integration::MoveToPoseGoal moveToPose_arm_template_goal;

// Template Goal Both Grippers arm
static integration::MoveToJointsGoal moveJoints_arm_template_goal;

// Template Goal Joints Kuka
static integration::MoveToJointsGoal moveJoints_gripper_template_goal;

// Template Human Model Move
static integration::MoveModelGoal moveHuman_template_goal;

// Template Attach Object
static integration::AttachObjectGoal attachObject_template_goal;

// Template Spawn Models
static integration::SpawnModels spawn_models_template_srv;

// Template Move Base
static move_base_msgs::MoveBaseGoal move_base_template_goal;

//=========================================================================//
//=========================================================================//
//=========================================================================//

//=========================================================================//
//==================== GOALS DEFINITION ===================================//
//=========================================================================//

//////////Goals Cartesian Arm
static integration::MoveToPoseGoal moveToPose_arm_safe_appr_goal, moveToPose_arm_appr_goal, moveToPose_arm_lift_goal,
    moveToPose_arm_pre_place_goal, moveToPose_arm_place_goal;
//////////Goals Joints Arm
static integration::MoveToJointsGoal moveJoints_arm_start_goal, moveJoints_arm_calibrated_goal,
    moveJoints_arm_out_of_view_goal, moveJoints_arm_for_guidance_goal;

//////////Goals Joints Gripper
static integration::MoveToJointsGoal moveJoints_gripper_open_goal, moveJoints_gripper_close_goal,
    moveJoints_gripper_semi_close_goal;

/////////Goals Human Move
static integration::MoveModelGoal moveHuman_test_goal;

///////Goals Attach Objects
static integration::AttachObjectGoal attachObject_ee_model_goal, detachObject_ee_model_goal;

// Services
static integration::GetFrameDistance get_target_position_srv;
static integration::AddBox add_box_test1_srv, remove_box_test1_srv;
static integration::SpawnModels spawn_models_test1_srv;

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
  moveToPose_arm_ac = new actionlib::SimpleActionClient<integration::MoveToPoseAction>("/move_cartesian_arm_"
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
  model_move_ac =
      new actionlib::SimpleActionClient<integration::MoveModelAction>("/model_move_handler_server/action", true);

  attach_object_ac =
      new actionlib::SimpleActionClient<integration::AttachObjectAction>("/object_attach_handler_server/action", true);

  move_base_ac = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("/move_base", true);
}

void deleteActionClients()
{
  delete moveToPose_arm_ac;
  delete model_move_ac;
  delete attach_object_ac;
  delete moveJoints_gripper_ac;
  delete moveJoints_arm_ac;
  delete move_base_ac;
}

void sigIntHandler(int sig)
{
  moveToPose_arm_ac->cancelAllGoals();
  attach_object_ac->cancelAllGoals();
  model_move_ac->cancelAllGoals();
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
  moveToPose_arm_ac->cancelAllGoals();
  moveJoints_gripper_ac->cancelAllGoals();
  moveJoints_arm_ac->cancelAllGoals();
  move_base_ac->cancelAllGoals();
  ROS_ERROR("Canceling all Goals");
}

void setConstantGoalValues()
{
  // moveToPose_arm template goal
  moveToPose_arm_template_goal.target_pose = empty_pose;
  moveToPose_arm_template_goal.target_pose.header.frame_id = "arm_footprint";
  moveToPose_arm_template_goal.constraint_mode = 0;
  moveToPose_arm_template_goal.velocity = standard_velocity;
  moveToPose_arm_template_goal.acceleration = standard_acceleration;
  moveToPose_arm_template_goal.timeout = 160;

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

  // moveHuman template goal
  moveHuman_template_goal.x = 0;
  moveHuman_template_goal.y = 0;
  moveHuman_template_goal.th = 0;
  moveHuman_template_goal.keep_ori = false;
  moveHuman_template_goal.enable_random = false;
  moveHuman_template_goal.rand_x = 0.0;
  moveHuman_template_goal.rand_y = 0.0;

  // attachObject template goal
  attachObject_template_goal.attach = true;
  attachObject_template_goal.model_name_1 = "robot";
  attachObject_template_goal.link_name_1 = "right_link_6";
  attachObject_template_goal.model_name_2 = "product";
  attachObject_template_goal.link_name_2 = "link";

  // spawn models template goal
  spawn_models_template_srv.request.delete_human = false;
  spawn_models_template_srv.request.product_id = "";
  spawn_models_template_srv.request.random = false;
  spawn_models_template_srv.request.spawn_human = true;

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
  setJointGoalValues(moveJoints_arm_out_of_view_goal, { 1.5353, -1.4187, 1.5429, 0.0, -1.57, 0.0 });

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
  
  // Pick Place Goals

  moveToPose_arm_safe_appr_goal = moveToPose_arm_template_goal;
  moveToPose_arm_safe_appr_goal.target_pose.header.frame_id = "box_red";
  moveToPose_arm_safe_appr_goal.constraint_mode = 0;
  setCartesianGoalValues(moveToPose_arm_safe_appr_goal, { -0.07, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 });

  moveToPose_arm_appr_goal = moveToPose_arm_template_goal;
  moveToPose_arm_appr_goal.target_pose.header.frame_id = "box_red";
  moveToPose_arm_appr_goal.constraint_mode = 0;
  setCartesianGoalValues(moveToPose_arm_appr_goal, { -0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 });

  moveToPose_arm_lift_goal = moveToPose_arm_template_goal;
  moveToPose_arm_lift_goal.target_pose.header.frame_id = "box_red";
  moveToPose_arm_lift_goal.constraint_mode = 0;
  setCartesianGoalValues(moveToPose_arm_lift_goal, { -0.02, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0 });

  //=========================================================================//
  //==================== ATTACH DETACH GOALS ================================//
  //=========================================================================//

  // Attach
  attachObject_ee_model_goal = attachObject_template_goal;
  attachObject_ee_model_goal.model_name_1 = "ommp_sim";
  attachObject_ee_model_goal.link_name_1 = "wrist_3_link";
  attachObject_ee_model_goal.model_name_2 = "box_red";
  attachObject_ee_model_goal.link_name_2 = "link";

  detachObject_ee_model_goal = attachObject_ee_model_goal;
  detachObject_ee_model_goal.attach = false;

  //=========================================================================//
  //==================== GET FRAME SERVICE REQUESTS==========================//
  //=========================================================================//

  get_target_position_srv.request.source_frame = "robot_footprint";
  get_target_position_srv.request.target_frame = "box_red";

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
  //==================== SPAWN MODELS SERVICE REQUESTS ========================//
  //=========================================================================//

  // spawn_models_start_srv = spawn_models_template_srv;
  //=========================================================================//
  //==================== MOVE BASE ==========================================//
  //=========================================================================//
  move_base_forward_goal = move_base_template_goal;
  move_base_forward_goal.target_pose.pose.position.x = 1.5;
  move_base_forward_goal.target_pose.pose.position.y = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
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

void updateGoals()
{
}

#endif  // AERNNOVA_CLIENT_H

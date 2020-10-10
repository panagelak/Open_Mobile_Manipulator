#include<ommp_moveit_interface/pick_place_server.h>

MyPickPlace::MyPickPlace(ros::NodeHandle nh)
  : nh_(nh),
    move_group(PLANNING_GROUP),
    gripper_group(GRIPPER_GROUP)
{
  

  // Pointer to JointModelGroup for improved performance.
  joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  gripper_joint_model_group =
    gripper_group.getCurrentState()->getJointModelGroup(GRIPPER_GROUP);

  visual_tools_ptr.reset(new moveit_visual_tools::MoveItVisualTools("robot_footprint"));
  visual_tools_ptr->deleteAllMarkers();
  visual_tools_ptr->loadRemoteControl();

  // Create text marker for displaying current state
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.7;
  text_pose.translation().x() = 0.3;
  visual_tools_ptr->publishText(text_pose, "Pick and Place",
                           rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);

  // Publish messages to rviz
  visual_tools_ptr->trigger();
  //visual_tools_ptr->prompt("Click Next");

  //start pos
  //move_group.setNamedTarget("out_of_view"); //out_of_view

  //success = move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;

}

bool MyPickPlace::Routine(ommp_moveit_interface::PickPlace::Request &req,
                      ommp_moveit_interface::PickPlace::Response &res)
{

  // test
  visual_tools_ptr->deleteAllMarkers();

  // Create text marker for displaying current state
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.7;
  text_pose.translation().x() = 0.3;
  visual_tools_ptr->publishText(text_pose, "New request received",
                           rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
  visual_tools_ptr->trigger();
  visual_tools_ptr->prompt("Click Next");

  // poses from request
  geometry_msgs::Pose dist_pose, pick_pose, lift_pose;
  std::vector<double> pre_place_pose, place_pose, away_pose;
  dist_pose = req.dist_pose;
  pick_pose = req.pick_pose;
  lift_pose = req.lift_pose;
  //place_pose = req.place_pose;

  // Plan arm motion
  // set starting pose
  

  // set safe distance pose
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(dist_pose);
  success = move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  ROS_INFO("Visualizing plan to target: %s",
            success ? "SUCCEEDED" : "FAILED");

  // We can also visualize the plan as a line with markers in Rviz.
  ROS_INFO("Visualizing plan to safe distance from object as trajectory line");
  visual_tools_ptr->deleteAllMarkers();
  visual_tools_ptr->publishAxisLabeled(dist_pose, "safe_distance_pose");
  visual_tools_ptr->publishText(text_pose, "Safe Distance From Object", rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
  visual_tools_ptr->publishTrajectoryLine(arm_plan.trajectory_, joint_model_group);
  visual_tools_ptr->trigger();
  visual_tools_ptr->prompt("Click Next");

  move_group.execute(arm_plan);

  //Close In movement
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(pick_pose);
  success = move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  ROS_INFO("Visualizing plan to target: %s",
            success ? "SUCCEEDED" : "FAILED");

  // We can also visualize the plan as a line with markers in Rviz.
  visual_tools_ptr->deleteAllMarkers();
  visual_tools_ptr->publishAxisLabeled(pick_pose, "pick_pose");
  visual_tools_ptr->publishText(text_pose, "Grasping Object Pose", rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
  visual_tools_ptr->publishTrajectoryLine(arm_plan.trajectory_, joint_model_group);
  visual_tools_ptr->trigger();
  visual_tools_ptr->prompt("Click Next");

  move_group.execute(arm_plan);


  //Close Gripper
  OperateGripper(true);
  ros::Duration(1.0).sleep();

  //Lift movement
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(lift_pose);
  success = move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  ROS_INFO("Visualizing plan to target: %s",
            success ? "SUCCEEDED" : "FAILED");

  // We can also visualize the plan as a line with markers in Rviz.
  visual_tools_ptr->deleteAllMarkers();
  visual_tools_ptr->publishAxisLabeled(lift_pose, "lift_pose");
  visual_tools_ptr->publishText(text_pose, "Lift Object Off the Ground", rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
  visual_tools_ptr->publishTrajectoryLine(arm_plan.trajectory_, joint_model_group);
  visual_tools_ptr->trigger();
  visual_tools_ptr->prompt("Click Next");

  move_group.execute(arm_plan);


  // Go to Pre Place Joint state pos

  moveit::core::RobotStatePtr arm_current_state =
  move_group.getCurrentState();
  //std::vector<double> place_pose;
  arm_current_state->copyJointGroupPositions(joint_model_group,
      pre_place_pose);
  pre_place_pose[0] = -1.5;
  pre_place_pose[1] = 0.0;
  pre_place_pose[2] = 1.5;
  pre_place_pose[3] = 0.0;
  pre_place_pose[4] = 0.0;
  pre_place_pose[5] = 0.0;
  move_group.setJointValueTarget(pre_place_pose);
  ros::Duration(1.0).sleep();
  //bool success = move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  
  success = move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

  ROS_INFO("Visualizing plan to target: %s",
            success ? "SUCCEEDED" : "FAILED");
  // We can also visualize the plan as a line with markers in Rviz.
  visual_tools_ptr->deleteAllMarkers();
  //visual_tools_ptr->publishAxisLabeled(pre_place_pose, "pre_place_pose");
  visual_tools_ptr->publishText(text_pose, "Pre Place Pose", rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
  visual_tools_ptr->publishTrajectoryLine(arm_plan.trajectory_, joint_model_group);
  visual_tools_ptr->trigger();
  visual_tools_ptr->prompt("Click Next");
  move_group.execute(arm_plan);


  // Go to Place Joint state pos
  
  arm_current_state =
  move_group.getCurrentState();
  //std::vector<double> place_pose;
  arm_current_state->copyJointGroupPositions(joint_model_group,
      place_pose);
  place_pose[0] = -1.5;
  place_pose[1] = 1.25;
  place_pose[2] = 1.5;
  place_pose[3] = 0.0;
  place_pose[4] = -1.3;
  place_pose[5] = 0.0;
  move_group.setJointValueTarget(place_pose);
  ros::Duration(1.0).sleep();
  //bool success = move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  success = move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

  ROS_INFO("Visualizing plan to target: %s",
            success ? "SUCCEEDED" : "FAILED");

  // We can also visualize the plan as a line with markers in Rviz.
  visual_tools_ptr->deleteAllMarkers();
  //visual_tools_ptr->publishAxisLabeled(place_pose, "place_pose");
  visual_tools_ptr->publishText(text_pose, "Place Object to Ground", rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
  visual_tools_ptr->publishTrajectoryLine(arm_plan.trajectory_, joint_model_group);
  visual_tools_ptr->trigger();
  visual_tools_ptr->prompt("Click Next");
  move_group.execute(arm_plan);

  //Open Gripper
  
  OperateGripper(false);
  ros::Duration(1.0).sleep();

  // Move away from Object

  arm_current_state =
  move_group.getCurrentState();
  //std::vector<double> place_pose;
  arm_current_state->copyJointGroupPositions(joint_model_group,
      away_pose);
  away_pose[0] = -1.5;
  away_pose[1] = 0.0;
  away_pose[2] = 1.5;
  away_pose[3] = 0.0;
  away_pose[4] = 0.0;
  away_pose[5] = 0.0;
  move_group.setJointValueTarget(away_pose);
  ros::Duration(1.0).sleep();
  //success = move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;

  success = move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  ROS_INFO("Visualizing plan to target: %s",
            success ? "SUCCEEDED" : "FAILED");

  // We can also visualize the plan as a line with markers in Rviz.
  visual_tools_ptr->deleteAllMarkers();
  //visual_tools_ptr->publishAxisLabeled(away_pose, "away_pose");
  visual_tools_ptr->publishText(text_pose, "Move Away From Object", rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
  visual_tools_ptr->publishTrajectoryLine(arm_plan.trajectory_, joint_model_group);
  visual_tools_ptr->trigger();
  visual_tools_ptr->prompt("Click Next");
  move_group.execute(arm_plan);

  //Go to Out of View position
  
  move_group.setNamedTarget("out_of_view");
  ros::Duration(1.0).sleep();
  //success = move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  
  success = move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  ROS_INFO("Visualizing plan to target: %s",
            success ? "SUCCEEDED" : "FAILED");
  // We can also visualize the plan as a line with markers in Rviz.
  visual_tools_ptr->deleteAllMarkers();
  //visual_tools_ptr->publishAxisLabeled(start_pose, "start_pose");
  visual_tools_ptr->publishText(text_pose, "Go to Out of View location", rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
  visual_tools_ptr->publishTrajectoryLine(arm_plan.trajectory_, joint_model_group);
  visual_tools_ptr->trigger();
  visual_tools_ptr->prompt("Click Next");
  move_group.execute(arm_plan);
  //success = move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;

  res.success = true;
  
}


bool MyPickPlace::OperateGripper(const bool &close_gripper)
{
  // RobotState contains the current position/velocity/acceleration data
  moveit::core::RobotStatePtr gripper_current_state =
    gripper_group.getCurrentState();

  // Next get the current set of joint values for the group.
  std::vector<double> gripper_joint_positions;
  gripper_current_state->copyJointGroupPositions(gripper_joint_model_group,
      gripper_joint_positions);

  ROS_DEBUG("No. of joints in eef_group: %zd", gripper_joint_positions.size());

  // Set finger joint values
  if (close_gripper)
  {
    gripper_joint_positions[0] = 0.02;
    gripper_joint_positions[1] = 0.02;
  }
  else
  {
    gripper_joint_positions[0] = 0.0;
    gripper_joint_positions[1] = 0.0;
  }

  gripper_group.setJointValueTarget(gripper_joint_positions);
  ros::Duration(1.5).sleep();

  bool success = gripper_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  return success;
}



tf::Quaternion MyPickPlace::RPYToQuaternion(float R, float P, float Y)
{
  tf::Matrix3x3 mat;
  mat.setEulerYPR(Y,P,R);

  tf::Quaternion quat;
  mat.getRotation(quat);

  return quat;
}

MyPickPlace::~MyPickPlace(){}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_pick_place_server");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(8);
  spinner.start();
  MyPickPlace my_pick_place(nh);
  ros::ServiceServer service = nh.advertiseService("pick_place_routine", &MyPickPlace::Routine, &my_pick_place);
  ros::waitForShutdown();
  return 0;
}

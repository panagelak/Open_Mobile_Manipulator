#include <signal.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <ommp_demo_client.h>

int executeTasks(ros::NodeHandle n)
{
  ros::Publisher activate_arm_pub;
  std_msgs::Int32 activate_msg, deactivate_msg;
  activate_msg.data = 1;
  deactivate_msg.data = 0;
  activate_arm_pub = n.advertise<std_msgs::Int32>("/activate_arm", 10, true);

  move_base_forward_goal.target_pose.header.stamp = ros::Time::now();
  if (!performAction(20, "Move FORWARD", move_base_ac, move_base_forward_goal))
    return -1;
  move_base_backward_goal.target_pose.header.stamp = ros::Time::now();
  if (!performAction(21, "Move BACKWARDD", move_base_ac, move_base_backward_goal))
    return -1;
  move_base_start_goal.target_pose.header.stamp = ros::Time::now();
  if (!performAction(22, "Move START", move_base_ac, move_base_start_goal))
    return -1;

  if (!pubTopic(1, "Activate Arm", activate_arm_pub, activate_msg))
    return -1;
  if (!performAction(2, "Move Arm to the Start Position", moveJoints_arm_ac, moveJoints_arm_start_goal))
    return -1;

  if (!performAction(3, "Move Arm to the Calibrated Position", moveJoints_arm_ac, moveJoints_arm_calibrated_goal))
    return -1;
  if (!performAction(4, "Move Arm to the Start Position", moveJoints_arm_ac, moveJoints_arm_start_goal))
    return -1;
  if (!pubTopic(5, "Deactivate Arm", activate_arm_pub, deactivate_msg))
    return -1;

  // if (!performAction(3, "Move Arm to the Guidance Position", moveJoints_arm_ac, moveJoints_arm_for_guidance_goal))
  //  return -1;
  //
  // if (!performAction(4, "Move Arm to the Calibrated Position", moveJoints_arm_ac, moveJoints_arm_calibrated_goal))
  //  return -1;
  //
  // if (!performAction(5, "Move Arm Outside the view of the KInect", moveJoints_arm_ac,
  // moveJoints_arm_out_of_view_goal))
  //  return -1;
  //
  // if (!performAction(6, "Move Arm to the Calibrated Position", moveJoints_arm_ac, moveJoints_arm_calibrated_goal))
  //  return -1;
  //
  // if (!performAction(7, "Open Grippers", moveJoints_gripper_ac, moveJoints_gripper_open_goal))
  //  return -1;
  //
  // if (!performAction(8, "Semi Close Grippers", moveJoints_gripper_ac, moveJoints_gripper_semi_close_goal))
  //  return -1;
  //
  // if (!performAction(9, "Close Grippers", moveJoints_gripper_ac, moveJoints_gripper_close_goal))
  //  return -1;
  //
  // if (!performAction(10, "Open Grippers", moveJoints_gripper_ac, moveJoints_gripper_open_goal))
  //  return -1;
  //
  // if (!performAction(11, "Move Arm Cartesian Down", moveCartesian_arm_ac, moveCartesian_arm_test1_goal))
  //  return -1;
  //
  // if (!performAction(12, "Move Arm Cartesian Up", moveCartesian_arm_ac, moveCartesian_arm_test2_goal))
  //  return -1;
  //
  // if (!performAction(13, "Move Arm Cartesian Down", moveCartesian_arm_ac, moveCartesian_arm_test1_goal))
  //  return -1;
  //
  // if (!performAction(14, "Move Arm Cartesian Down", moveCartesian_arm_ac, moveCartesian_arm_test1_goal))
  //  return -1;
  //
  // if (!performAction(15, "Move Arm Cartesian Up", moveCartesian_arm_ac, moveCartesian_arm_test2_goal))
  //  return -1;
  //
  // if (!performAction(16, "Move Arm Cartesian Up", moveCartesian_arm_ac, moveCartesian_arm_test2_goal))
  //  return -1;
  //
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ommp_demo_node", ros::init_options::NoSigintHandler);

  ros::NodeHandle n("");
  ros::NodeHandle n_local("~");

  signal(SIGINT, sigIntHandler);

  // current_task_publisher =
  //    n.advertise<integration::ScheduleExecutorTaskStatus>("/schedule_executor/task_status", 10, true);

  boost::thread spin_thread(mySpin);

  createActionClients();
  // Create Safety Zones Background
  // ros::Subscriber in_zones_sub_ = n.subscribe<integration::BoolArray>("/in_safety_zones", 10, in_zonesCB);

  // Create Service Clients
  ros::ServiceClient get_frame_distance_client = n.serviceClient<integration::GetFrameDistance>("/get_frame_"
                                                                                                "distance");
  get_frame_distance_clientPrt = &get_frame_distance_client;
  ros::ServiceClient add_box_client = n.serviceClient<integration::AddBox>("/add_planning_box_routine");
  add_box_clientPrt = &add_box_client;

  printLogo();

  while (!(first <= 200 && first >= 0 && last <= 65 && last >= 0 && first <= last))
  {
    ROS_INFO("Enter first and last step IDs (1 - 65)");
    std::cin >> first >> last;
    if (first == -1 and last == -1)
      return -1;
  }

  printTitle("OMMP DEMO STARTED");

  setConstantGoalValues();
  setGoalValues();

  // Begin demo
  execution_result = executeTasks(n_local);

  spin_thread.join();
  deleteActionClients();

  return execution_result;
}
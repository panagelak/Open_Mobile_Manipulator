#include <signal.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <ommp_demo_client.h>

int executeTasks(ros::NodeHandle n)
{
  move_base_forward_goal.target_pose.header.stamp = ros::Time::now();
  if (!performAction(1, "Move FORWARD", move_base_ac, move_base_forward_goal))
    return -1;

  if (!performAction(2, "Move Arm to the Start Position", moveJoints_arm_ac, moveJoints_arm_start_goal))
    return -1;

  if (!performAction(5, "Move Arm Outside the view of the KInect", moveJoints_arm_ac, moveJoints_arm_out_of_view_goal))
    return -1;
  
  ros::Publisher kinect_pub = n.advertise<std_msgs::Float64>("kinect_controller/command",10);
  std_msgs::Float64 k_msg;
  k_msg.data = 0.62;
  kinect_pub.publish(k_msg);

  ros::Duration(5).sleep();
  if (!callService(6,"Get frame Distance for target",get_frame_distance_clientPrt,&get_target_position_srv));
  
  updateGoals();

  if (!performAction(7, "Open Grippers", moveJoints_gripper_ac, moveJoints_gripper_open_goal))
    return -1;

  if (!performAction(8, "Semi Close Grippers", moveJoints_gripper_ac, moveJoints_gripper_semi_close_goal))
    return -1;


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
  ros::ServiceClient spawn_models_client = n.serviceClient<integration::SpawnModels>("/spawn_models_routine");
  spawn_models_clientPrt = &spawn_models_client;

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
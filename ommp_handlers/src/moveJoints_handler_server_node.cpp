#include <ros/ros.h>
#include <sys/ioctl.h>  // For ioctl, TIOCGWINSZ
#include <unistd.h>     // For STDOUT_FILENO

#include "ommp_handlers/NamingConstants.h"

// action libs
#include <actionlib/server/simple_action_server.h>
#include <integration/ActionResultStatusConstants.h>
#include <integration/MoveToJointsAction.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <boost/thread/thread.hpp>
// #include <moveit/planning_interface/planning_interface.h>
#include <integration/ActionResultStatusConstants.h>
#include <integration/MoveToPoseAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// For Stopping
#include <control_msgs/FollowJointTrajectoryAction.h>
// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <sensor_msgs/JointState.h>
#include <boost/shared_ptr.hpp>

class MoveJointsHandlerServer
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  actionlib::SimpleActionServer<integration::MoveToJointsAction> as_;
  std::string action_name_;
  integration::MoveToJointsFeedback feedback_;
  integration::MoveToJointsResult result_;

  std::string PLANNING_GROUP_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::planning_interface::MoveGroupInterface::Plan plan_;

  bool moveit_running = true, planning_succeeded_, execution_succeeded_, plannig_thread_done_, execution_thread_done_;

  std::string ROBOT_;
  const std::vector<std::string> AvailableJointNames_arm = { "shoulder_pan_joint", "shoulder_lift_joint",
                                                             "elbow_joint",        "wrist_1_joint",
                                                             "wrist_2_joint",      "wrist_3_joint" };

  const std::vector<std::string> AvailableJointNames_gripper = { "finger1_joint", "finger2_joint" };

  // For stopping the Robot - Emergency
  control_msgs::FollowJointTrajectoryGoal stop_goal;
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
  typedef boost::shared_ptr<arm_control_client> arm_control_client_Ptr;
  arm_control_client_Ptr ArmClient;

public:
  MoveJointsHandlerServer(ros::NodeHandle &nh, ros::NodeHandle &nh_local, std::string name)
    : nh_(nh)
    , nh_local_(nh_local)
    , as_(nh_, name, boost::bind(&MoveJointsHandlerServer::executeCallback, this, _1), false)
    , action_name_(name)
  {
    nh_local_.getParam("move_group", PLANNING_GROUP_);
    nh_local_.getParam("robot", ROBOT_);

    const std::shared_ptr<tf2_ros::Buffer> dummy_tf;
    // const boost::shared_ptr<tf::Transformer> dummy_tf;

    ROS_INFO("Waiting for MoveIt servers to respond...");
    try
    {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_, dummy_tf,
                                                                                     ros::Duration(50));
    }
    catch (const std::runtime_error &e)
    {
      ROS_ERROR("[%s] %s Continuing without MoveIt planning option.", action_name_.c_str(), e.what());
      moveit_running = false;
    }
    as_.start();
    ROS_INFO("[%s] Ready to receive goals.", action_name_.c_str());
    // For Emergency
    // Create an arm controller action client to move the arm For Emergency
    createArmClient(ArmClient);
    configure_arm_goal(stop_goal);

  }

  ~MoveJointsHandlerServer(void)
  {
  }
  // For Emergency
  // Create a ROS action client to move arm
  void createArmClient(arm_control_client_Ptr &actionClient)
  {
    ROS_INFO("Creating action client to arm controller ...");
    if (ROBOT_ == "arm")
      actionClient.reset(new arm_control_client("/arm_controller/follow_joint_trajectory"));
    if (ROBOT_ == "gripper")
      actionClient.reset(new arm_control_client("/gripper_controller/follow_joint_trajectory"));

    int iterations = 0, max_iterations = 3;
    // Wait for arm controller action server to come up
    while (!actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations)
    {
      ROS_DEBUG("Waiting for the arm_controller_action server to come up");
      ++iterations;
    }

    if (iterations == max_iterations)
      throw std::runtime_error("Error in createArmClient: arm controller action server not available");
  }
  // For Emergency
  // Pre configure the emergency goal for faster response
  void configure_arm_goal(control_msgs::FollowJointTrajectoryGoal &goal)
  {
    for (std::string name : move_group_->getJointNames())
      goal.trajectory.joint_names.push_back(name);
    int size = move_group_->getActiveJoints().size();
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(size);
    goal.trajectory.points[0].velocities.resize(size);
    goal.trajectory.points[0].accelerations.resize(size);
    for (int i = 0; i < size; i++)
    {
      goal.trajectory.points[0].velocities[i] = 1.0;
    }
    for (int i = 0; i < size; i++)
    {
      goal.trajectory.points[0].accelerations[i] = 1.0;
    }
    goal.trajectory.points[0].time_from_start = ros::Duration(0.1);
  }
  // Generates a simple trajectory with the current waypoint to move arm
  void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal &goal)
  {
    // Get Current Joint Positions (from the whole robot)
    std::vector<double> cur_pos = move_group_->getCurrentJointValues();
    int size = move_group_->getActiveJoints().size();
    // Current Position trajectory
    for (int i = 0; i < size; i++)
    {
      goal.trajectory.points[0].positions[i] = cur_pos[i];
    }
  }
  void executeCallback(const integration::MoveToJointsGoalConstPtr &goal)
  {
    ros::WallTime _start;
    _start = ros::WallTime::now();  // Start timer
    ROS_INFO("[%s] MoveJoints goal received", action_name_.c_str());

    for (integration::PropertyValuePair joint : goal->joints)
      if (!checkJointName(joint.name))
      {
        ROS_ERROR("[%s] Could not find joint: \"%s\". Unable to send goal.", action_name_.c_str(), joint.name.c_str());
        feedback_.action_feedback.millisPassed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
        feedback_.action_feedback.status = "Cannot process goal";
        as_.publishFeedback(feedback_);
        ros::Duration(0.1).sleep();
        result_.action_result.success = false;
        result_.action_result.status = integration::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
        result_.action_result.millisPassed = feedback_.action_feedback.millisPassed;
        as_.setAborted(result_);
        printLine();
        return;
      }

    if (!moveit_running)
    {
      ROS_ERROR("Trying to plan goal with MoveIt but MoveIt is not running.");
      feedback_.action_feedback.millisPassed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
      // feedback_.state = "Cannot process goal";
      as_.publishFeedback(feedback_);
      ros::Duration(0.1).sleep();
      result_.action_result.success = false;
      result_.action_result.status = integration::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
      result_.action_result.millisPassed = feedback_.action_feedback.millisPassed;
      as_.setAborted(result_);
      printLine();
      return;
    }

    ROS_INFO("[%s] Planning frame: %s", action_name_.c_str(), move_group_->getPlanningFrame().c_str());
    ROS_INFO("[%s] End effector link: %s", action_name_.c_str(), move_group_->getEndEffectorLink().c_str());
    std::cout << "\n";

    moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();

    const robot_state::JointModelGroup *joint_model_group_ =
        move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP_);

    std::vector<double> joint_group_positions_;
    current_state->copyJointGroupPositions(joint_model_group_, joint_group_positions_);
    const std::vector<std::string> joint_group_names_ = move_group_->getJointNames();
    std::map<std::string, double> name_value_pair =
        createNameValueMap(joint_group_names_, joint_group_positions_, goal->joints);

    move_group_->setMaxVelocityScalingFactor(double(goal->endEffectorVelocity));
    move_group_->setMaxAccelerationScalingFactor(double(goal->endEffectorAcceleration));
    move_group_->setPlanningTime(double(goal->timeoutSeconds));

    move_group_->setJointValueTarget(name_value_pair);

    plannig_thread_done_ = false;
    boost::thread planning_thread(&MoveJointsHandlerServer::moveit_planning, this);
    while (!plannig_thread_done_)
    {
      ros::Duration(0.001).sleep();
      feedback_.action_feedback.millisPassed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
      // feedback_.state = "Planning...";
      as_.publishFeedback(feedback_);

      if (as_.isPreemptRequested() || !ros::ok())
      {
        move_group_->stop();
        feedback_.action_feedback.millisPassed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
        // feedback_.state = "Planning was Stoped";
        as_.publishFeedback(feedback_);
        ros::Duration(0.1).sleep();  // Wait for feedback to be updated before publishing the result
        ROS_WARN("[%s] Preempted", action_name_.c_str());
        result_.action_result.success = false;
        result_.action_result.status = integration::ActionResultStatusConstants::CANCELLED;
        result_.action_result.millisPassed = feedback_.action_feedback.millisPassed;
        as_.setPreempted(result_);
        printLine();
        return;
      }
    }

    if (!planning_succeeded_)
    {
      ROS_ERROR("[%s] Planning joint goal FAILED", action_name_.c_str());
      feedback_.action_feedback.millisPassed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
      // feedback_.state = "Planning failed";
      as_.publishFeedback(feedback_);
      ros::Duration(0.1).sleep();  // Wait for feedback to be updated before publishing the result
      result_.action_result.success = false;
      result_.action_result.status = integration::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
      result_.action_result.millisPassed = feedback_.action_feedback.millisPassed;
      as_.setAborted(result_);
      return;
    }
    else
    {
      ROS_INFO("[%s] Planning joint goal SUCCEEDED", action_name_.c_str());

      execution_thread_done_ = false;
      boost::thread execution_thread(&MoveJointsHandlerServer::moveit_execution, this);
      while (!execution_thread_done_)
      {
        ros::Duration(0.001).sleep();

        feedback_.action_feedback.millisPassed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
        // feedback_.state = "Executing trajectory...";
        as_.publishFeedback(feedback_);

        if (as_.isPreemptRequested() || !ros::ok())
        {
          move_group_->stop();
          feedback_.action_feedback.millisPassed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
          ////// Send Goal to follow_joint_trajectory/action with current joints
          waypoints_arm_goal(stop_goal);
          stop_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.01);
          ArmClient->sendGoal(stop_goal);
          // Wait for trajectory execution
          while (!(ArmClient->getState().isDone()) && ros::ok())
          {
            ros::Duration(0.1).sleep();  // sleep for four seconds
          }
          //////////////////////////////////////
          // feedback_.state = "Execution was Stoped";
          as_.publishFeedback(feedback_);
          ros::Duration(0.1).sleep();  // Wait for feedback to be updated before publishing the result
          ROS_WARN("[%s] Preempted", action_name_.c_str());
          result_.action_result.success = false;
          result_.action_result.status = integration::ActionResultStatusConstants::CANCELLED;
          result_.action_result.millisPassed = feedback_.action_feedback.millisPassed;

          as_.setPreempted(result_);
          printLine();
          return;
        }
      }

      if (execution_succeeded_)
      {
        ROS_INFO("[%s] Succeeded", action_name_.c_str());
        // feedback_.state = "Trajectory execution completed";
        feedback_.action_feedback.millisPassed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
        as_.publishFeedback(feedback_);
        ros::Duration(0.1).sleep();  // Wait for feedback to be updated before publishing the result
        result_.action_result.success = true;
        result_.action_result.status = integration::ActionResultStatusConstants::SUCCESS;
        result_.action_result.millisPassed = feedback_.action_feedback.millisPassed;

        as_.setSucceeded(result_);
      }
      else
      {
        // TODO: Sepcify any exceptions
        // feedback_.state = "Trajectory execution failed";
        feedback_.action_feedback.millisPassed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
        as_.publishFeedback(feedback_);
        ros::Duration(0.1).sleep();  // Wait for feedback to be updated before publishing the result
        result_.action_result.success = false;
        result_.action_result.status = integration::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
        result_.action_result.millisPassed = feedback_.action_feedback.millisPassed;
        as_.setAborted(result_);
      }
    }
  }

  bool checkJointName(const std::string &name)
  {
    if (ROBOT_ == "arm")
    {
      for (std::string available : AvailableJointNames_arm)
        if (name == available)
          return true;
      return false;
    }
    if (ROBOT_ == "gripper")
    {
      for (std::string available : AvailableJointNames_gripper)
        if (name == available)
          return true;
      return false;
    }
    return false;
  }

  std::map<std::string, double> createNameValueMap(const std::vector<std::string> &names,
                                                   const std::vector<double> &values,
                                                   const std::vector<integration::PropertyValuePair> &goal)
  {
    std::map<std::string, double> map;
    for (uint i = 0; i < names.size(); i++)
    {
      for (uint j = 0; j < goal.size(); j++)
      {
        if (names[i] == goal[j].name)
        {
          map.insert({ goal[j].name, goal[j].value });
          break;
        }
        else if (j == goal.size() - 1)
          map.insert({ names[i], values[i] });
      }
    }
    return map;
  }

  void moveit_planning()
  {
    planning_succeeded_ = (move_group_->plan(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    plannig_thread_done_ = true;
  }

  void moveit_execution()
  {
    execution_succeeded_ = (move_group_->execute(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    execution_thread_done_ = true;
  }

  void printLine()
  {
    struct winsize size;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &size);
    for (uint i = 0; i < size.ws_col; i++)
      std::cout << "=";
    std::cout << "\n";
  }
};

int main(int argc, char **argv)
{
  const std::string server_name = "_move_joints_handler_server";
  const std::string server_node_name = server_name + SERVER_NODE_NAME_POSTFIX;
  const std::string server_action_name = server_name + SERVER_ACTION_NAME_POSTFIX;

  ros::init(argc, argv, server_node_name, ros::init_options::NoRosout);

  std::string handler_prefix = "";
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");
  nh_local.getParam("prefix", handler_prefix);

  try
  {
    ROS_INFO_STREAM("[" << handler_prefix + "" + server_node_name << "]"
                        << "Initializing node with ns :" << nh.getNamespace());
    MoveJointsHandlerServer handler(nh, nh_local, handler_prefix + "" + server_action_name);
    ros::spin();
  }
  catch (const char *s)
  {
    ROS_FATAL_STREAM("[" << handler_prefix + "" + server_node_name << "]" << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[" << handler_prefix + "" + server_node_name << "]"
                         << "Unexpected error");
  }
}
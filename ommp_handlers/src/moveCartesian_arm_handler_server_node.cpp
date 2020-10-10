#include <ros/ros.h>
#include <sys/ioctl.h>  // For ioctl, TIOCGWINSZ
#include <unistd.h>     // For STDOUT_FILENO

#include "ommp_handlers/NamingConstants.h"

// action libs
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <integration/ActionResultStatusConstants.h>
#include <integration/MoveToPoseAction.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

// moveit libs
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <math.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <boost/thread/thread.hpp>

// For Visualization
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// For Stopping
#include <control_msgs/FollowJointTrajectoryAction.h>
// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <sensor_msgs/JointState.h>
#include <boost/shared_ptr.hpp>


class MoveCartesianHandlerServer
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  actionlib::SimpleActionServer<integration::MoveToPoseAction> as_;
  std::string action_name_;
  integration::MoveToPoseFeedback feedback_;
  integration::MoveToPoseResult result_;

  std::string PLANNING_GROUP_, namespc_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::planning_interface::MoveGroupInterface::Plan plan_;

  bool moveit_running = true, planning_succeeded_, execution_succeeded_, plannig_thread_done_, execution_thread_done_;

  ros::WallTime _start;
  // Visualization
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_ptr;
  const robot_state::JointModelGroup *joint_model_group;
  // For stopping the Robot - Emergency
  control_msgs::FollowJointTrajectoryGoal stop_goal;
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
  typedef boost::shared_ptr<arm_control_client> arm_control_client_Ptr;
  arm_control_client_Ptr ArmClient;

public:
  MoveCartesianHandlerServer(ros::NodeHandle &nh, ros::NodeHandle &nh_local, std::string name)
    : nh_(nh)
    , nh_local_(nh_local)
    , as_(nh_, name, boost::bind(&MoveCartesianHandlerServer::executeCallback, this, _1), false)
    , action_name_(name)
  {
    nh_local_.getParam("move_group", PLANNING_GROUP_);
    nh_local_.getParam("prefix", namespc_);

    const std::shared_ptr<tf2_ros::Buffer> dummy_tf;

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
    // Visualization
    visual_tools_ptr.reset(new moveit_visual_tools::MoveItVisualTools("robot_footprint"));
    visual_tools_ptr->loadRemoteControl();
    as_.start();
    ROS_INFO("[%s] Ready to receive goals.", action_name_.c_str());
    // For Emergency
    // Create an arm controller action client to move the arm For Emergency
    createArmClient(ArmClient);
    configure_arm_goal(stop_goal);

  }

  ~MoveCartesianHandlerServer(void)
  {
  }
  // For Emergency
  // Create a ROS action client to move arm
  void createArmClient(arm_control_client_Ptr &actionClient)
  {
    ROS_INFO("Creating action client to arm controller ...");

    actionClient.reset(new arm_control_client("/arm_controller/follow_joint_trajectory"));
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
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(6);
    goal.trajectory.points[0].velocities.resize(6);
    goal.trajectory.points[0].accelerations.resize(6);
    for (int i = 0; i < 6; i++)
    {
      goal.trajectory.points[0].velocities[i] = 1.0;
    }
    for (int i = 0; i < 6; i++)
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
    // Current Position trajectory
    for (int i = 0; i < 6; i++)
    {
      goal.trajectory.points[0].positions[i] = cur_pos[i];
    }
  }

  void executeCallback(const integration::MoveToPoseGoalConstPtr &goal)
  {
    // ros::WallTime _start;
    _start = ros::WallTime::now();  // Start timer
    ROS_INFO("[%s] MoveCartesian goal received", action_name_.c_str());

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
    printf("\n");
    move_group_->setMaxVelocityScalingFactor(double(goal->velocity));
    move_group_->setMaxAccelerationScalingFactor(double(goal->acceleration));
    move_group_->setPlanningTime(double(goal->timeout));

    geometry_msgs::PoseStamped transformed_pose = changePoseFrame("arm_footprint", goal->target_pose);
    if (transformed_pose.header.frame_id == "wrist_3_link")
    {
      feedback_.action_feedback.millisPassed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
      // feedback_.state = "Cannot process goal";
      as_.publishFeedback(feedback_);
      ros::Duration(0.1).sleep();  // Wait for feedback to be updated before publishing the result
      result_.action_result.success = false;
      result_.action_result.status = integration::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
      result_.action_result.millisPassed = feedback_.action_feedback.millisPassed;
      ROS_WARN("[%s] Set Aborted from frameID ", action_name_.c_str());
      as_.setAborted(result_);
      printLine();
      return;
    }
    else
    {
      move_group_->setPoseTarget(transformed_pose);
    }
    //*/////////// Define Motion Constraints
    if (goal->constraint_mode == 1)
    {
      tf2::Quaternion quat;
      quat.setRPY(-1.57, 1.57, 0);  // M_PI
      quat.normalize();

      moveit_msgs::OrientationConstraint ocm;

      ocm.link_name = "wrist_3_link";
      ocm.header.frame_id = "arm_footprint";
      ocm.orientation.x = 0.5;   //-quat[0]; //0
      ocm.orientation.y = -0.5;  //-quat[1];  //0
      ocm.orientation.z = -0.5;  //-quat[2];  //0
      ocm.orientation.w = -0.5;  //-quat[3];  //1
      ocm.absolute_x_axis_tolerance = 0.2;
      ocm.absolute_y_axis_tolerance = 0.2;
      ocm.absolute_z_axis_tolerance = 0.2;
      ocm.weight = .6;

      // Now, set it as the path constraint for the group.
      moveit_msgs::Constraints ori_constraints;
      ori_constraints.orientation_constraints.push_back(ocm);
      move_group_->setPathConstraints(ori_constraints);
    }
    //*/////////// END Define Constraint

    plannig_thread_done_ = false;
    boost::thread planning_thread(&MoveCartesianHandlerServer::moveit_planning, this);
    while (!plannig_thread_done_)
    {
      ros::Duration(0.001).sleep();
      feedback_.action_feedback.millisPassed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
      // feedback_.state = "Planning...";
      as_.publishFeedback(feedback_);

      if (as_.isPreemptRequested() || !ros::ok())
      {
        move_group_->stop();
        if (goal->constraint_mode)
          move_group_->clearPathConstraints();

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
      if (goal->constraint_mode)
        move_group_->clearPathConstraints();
      ROS_ERROR("[%s] Planning joint goal FAILED", action_name_.c_str());
      feedback_.action_feedback.millisPassed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
      // feedback_.state = "Planning failed";
      as_.publishFeedback(feedback_);
      ros::Duration(0.1).sleep();  // Wait for feedback to be updated before publishing the result
      ROS_ERROR("[%s]: Aborted. No plan found.", action_name_.c_str());
      result_.action_result.success = false;
      result_.action_result.status = integration::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
      result_.action_result.millisPassed = feedback_.action_feedback.millisPassed;
      as_.setAborted(result_);
      return;
    }
    else
    {
      ROS_INFO("[%s] Planning joint goal SUCCEEDED", action_name_.c_str());
      // Visualization
      visual_tools_ptr->deleteAllMarkers();
      ROS_INFO("Visualizing plan to target: %s", planning_succeeded_ ? "SUCCEEDED" : "FAILED");
      joint_model_group = move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP_);
      visual_tools_ptr->publishTrajectoryLine(plan_.trajectory_, joint_model_group);
      visual_tools_ptr->trigger();

      execution_thread_done_ = false;
      boost::thread execution_thread(&MoveCartesianHandlerServer::moveit_execution, this);
      while (!execution_thread_done_)
      {
        ros::Duration(0.001).sleep();
        feedback_.action_feedback.millisPassed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
        // feedback_.state = "Executing Trajectory...";
        as_.publishFeedback(feedback_);

        if (as_.isPreemptRequested() || !ros::ok())
        {
          move_group_->stop();
          if (goal->constraint_mode)
            move_group_->clearPathConstraints();
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
          feedback_.action_feedback.millisPassed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
          // feedback_.state = "Execution was Stoped";
          as_.publishFeedback(feedback_);
          ros::Duration(0.1).sleep();  // Wait for feedback to be updated before publishing the result
          ROS_WARN("[%s]: Preempted", action_name_.c_str());
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
        if (goal->constraint_mode)
          move_group_->clearPathConstraints();
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
        if (goal->constraint_mode)
          move_group_->clearPathConstraints();
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

    printLine();
  }

  void moveit_planning()
  {
    move_group_->setStartStateToCurrentState();
    planning_succeeded_ = (move_group_->plan(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    plannig_thread_done_ = true;
  }

  void moveit_execution()
  {
    execution_succeeded_ = (move_group_->execute(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    execution_thread_done_ = true;
  }

  geometry_msgs::PoseStamped changePoseFrame(const std::string &target_frame,
                                             const geometry_msgs::PoseStamped &goal_pose)
  {
    tf2_ros::Buffer br;
    br.setUsingDedicatedThread(true);
    tf2_ros::TransformListener tf2_listener(br);
    geometry_msgs::TransformStamped transform;
    geometry_msgs::PoseStamped transformed_pose;

    try
    {
      transform = br.lookupTransform(target_frame, goal_pose.header.frame_id, ros::Time(0), ros::Duration(1.0));
      tf2::doTransform(goal_pose, transformed_pose, transform);
      ROS_INFO_STREAM("Change transform pose to..   " << transformed_pose);
      return transformed_pose;
    }
    catch (tf2::LookupException &e)
    {
      ROS_ERROR("[%s] %s", action_name_.c_str(), e.what());
      transformed_pose.header.frame_id = "wrist_3_link";
      return transformed_pose;
    }
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
  const std::string server_name = "move_cartesian_arm_handler_server";
  const std::string server_node_name = server_name + SERVER_NODE_NAME_POSTFIX;
  const std::string server_action_name = server_name + SERVER_ACTION_NAME_POSTFIX;

  ros::init(argc, argv, server_node_name, ros::init_options::NoRosout);

  std::string handler_prefix;
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");
  nh_local.getParam("prefix", handler_prefix);

  try
  {
    ROS_INFO_STREAM("[" << handler_prefix + "" + server_node_name << "]"
                        << "Initializing node with ns :" << nh.getNamespace());
    MoveCartesianHandlerServer handler(nh, nh_local, handler_prefix + "" + server_action_name);
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
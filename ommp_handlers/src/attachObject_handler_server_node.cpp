#include <ros/ros.h>
#include <sys/ioctl.h>  // For ioctl, TIOCGWINSZ
#include <unistd.h>     // For STDOUT_FILENO

#include "ommp_handlers/NamingConstants.h"

#include <actionlib/server/simple_action_server.h>
#include <integration/ActionResultStatusConstants.h>
#include <integration/AttachObjectAction.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "gazebo_ros_link_attacher/Attach.h"

class AttachObjectServerHandler
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  actionlib::SimpleActionServer<integration::AttachObjectAction> as_;
  std::string action_name_;
  integration::AttachObjectFeedback feedback_;
  integration::AttachObjectResult result_;

  ros::ServiceClient attach_client_;
  ros::ServiceClient detach_client_;
  gazebo_ros_link_attacher::Attach attach_srv_;
  gazebo_ros_link_attacher::Attach detach_srv_;

public:
  AttachObjectServerHandler(ros::NodeHandle& nh, ros::NodeHandle& nh_local, std::string name)
    : nh_(nh)
    , nh_local_(nh_local)
    , as_(nh_, name, boost::bind(&AttachObjectServerHandler::executeCallback, this, _1), false)
    , action_name_(name)
  {
    attach_client_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    detach_client_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
    as_.start();
  }

  ~AttachObjectServerHandler(){};

  void executeCallback(const integration::AttachObjectGoalConstPtr& goal)
  {
    ros::WallTime _start;
    _start = ros::WallTime::now();  // Start timer

    if (goal->attach)
    {
      ROS_INFO("Receive Attach Object Goal");
      attach_srv_.request.model_name_1 = goal->model_name_1;
      attach_srv_.request.link_name_1 = goal->link_name_1;
      attach_srv_.request.model_name_2 = goal->model_name_2;
      attach_srv_.request.link_name_2 = goal->link_name_2;

      if (attach_client_.call(attach_srv_))
      {
        ROS_INFO("[%s]: Succeeded", action_name_.c_str());
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
        ROS_INFO("[%s]: Failed", action_name_.c_str());
        feedback_.action_feedback.millisPassed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
        as_.publishFeedback(feedback_);
        ros::Duration(0.1).sleep();  // Wait for feedback to be updated before publishing the result
        result_.action_result.success = false;
        result_.action_result.status = integration::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
        result_.action_result.millisPassed = feedback_.action_feedback.millisPassed;
        as_.setAborted(result_);
      }
    }
    else
    {
      ROS_INFO("Receive Detach Object Goal");
      detach_srv_.request.model_name_1 = goal->model_name_1;
      detach_srv_.request.link_name_1 = goal->link_name_1;
      detach_srv_.request.model_name_2 = goal->model_name_2;
      detach_srv_.request.link_name_2 = goal->link_name_2;

      if (detach_client_.call(detach_srv_))
      {
        ROS_INFO("[%s]: Succeeded", action_name_.c_str());
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
        ROS_INFO("[%s]: Failed", action_name_.c_str());
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

  void printLine()
  {
    struct winsize size;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &size);
    for (uint i = 0; i < size.ws_col; i++)
      std::cout << "=";
    std::cout << "\n";
  }
};

int main(int argc, char** argv)
{
  const std::string server_name = "object_attach_handler_server";
  const std::string server_node_name = server_name + SERVER_NODE_NAME_POSTFIX;
  const std::string server_action_name = server_name + SERVER_ACTION_NAME_POSTFIX;

  ros::init(argc, argv, server_node_name, ros::init_options::NoRosout);
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  try
  {
    ROS_INFO_STREAM("[" << server_node_name << "]"
                        << "Initializing node with ns :" << nh.getNamespace());
    AttachObjectServerHandler handler(nh, nh_local, server_action_name);
    ros::spin();
  }
  catch (const char* s)
  {
    ROS_FATAL_STREAM("[" << server_node_name << "]" << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[" << server_node_name << "]"
                         << "Unexpected error");
  }
}

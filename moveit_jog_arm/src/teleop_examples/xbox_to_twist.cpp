#include "geometry_msgs/TwistStamped.h"
#include "control_msgs/JointJog.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

namespace to_twist
{
class xboxToTwist
{
public:
  xboxToTwist() : spinner_(1)
  {
    joy_sub_ = n_.subscribe("joy", 1, &xboxToTwist::joyCallback, this);
    twist_pub_ = n_.advertise<geometry_msgs::TwistStamped>("jog_server/delta_jog_cmds", 1);
    joint_delta_pub_ = n_.advertise<control_msgs::JointJog>("jog_server/joint_delta_jog_cmds", 1);

    spinner_.start();
    ros::waitForShutdown(); 
  };

private:
  ros::NodeHandle n_;
  ros::Subscriber joy_sub_;
  ros::Publisher twist_pub_, joint_delta_pub_;
  ros::AsyncSpinner spinner_;

  // Convert incoming joy commands to TwistStamped commands for jogging
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
  {
    // Cartesian jogging
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();
    // This button is binary
    twist.twist.linear.x = -msg->buttons[4] + msg->buttons[5];
    // Double buttons
    twist.twist.linear.y = msg->axes[0];
    twist.twist.linear.z = msg->axes[1];
    twist.twist.angular.x = -msg->axes[3];
    twist.twist.angular.y = msg->axes[4];
    // A binary button
    twist.twist.angular.z = -msg->buttons[0] + msg->buttons[1]; // yaw

    // Joint jogging
    control_msgs::JointJog joint_deltas;
    // This example is for a UR5.
    joint_deltas.joint_names.push_back("shoulder_pan_joint");
    // Button 6: positive
    // Button 7: negative
    joint_deltas.velocities.push_back(msg->buttons[6] - msg->buttons[7]);
    joint_deltas.header.stamp = ros::Time::now();

    twist_pub_.publish(twist);
    joint_delta_pub_.publish(joint_deltas);
  }
};
}  // end to_twist namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xbox_to_twist");

  to_twist::xboxToTwist to_twist;

  return 0;
}

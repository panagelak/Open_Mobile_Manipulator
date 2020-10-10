#include <ros/ros.h>

#include <std_msgs/Float32.h>

#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"

#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32MultiArray.h"

class velHandler {
public:
  velHandler() {
    cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &velHandler::cmd_velCB, this);
    set_vel_pub = nh.advertise<std_msgs::Float32MultiArray>("set_vel", 10);
  }

  void cmd_velCB(const geometry_msgs::Twist::ConstPtr &cmd) {
    std_msgs::Float32MultiArray array;
    array.data.clear();
    float x = cmd->linear.x;
    float z = cmd->angular.z;
    float left_vel = x + (z * w / 2.0) / 0.06;
    float right_vel = x - (z * w / 2.0) / 0.06;

    array.data.push_back(left_vel);
    array.data.push_back(right_vel);

    set_vel_pub.publish(array);
  }

protected:
  ros::NodeHandle nh;
  ros::Subscriber cmd_vel_sub;
  ros::Publisher set_vel_pub;
  float w = 0.25;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "vel_to_arduino");
  velHandler handler;
  ros::spin();
  return 0;
}

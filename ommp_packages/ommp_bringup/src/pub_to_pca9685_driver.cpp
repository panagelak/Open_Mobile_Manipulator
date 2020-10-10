
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include <math.h>

#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
class arduinoHandler {
public:
  arduinoHandler() {
    js_sub = nh.subscribe("/joint_states", 10, &arduinoHandler::jsCB, this);
    activate_sub = nh.subscribe("/activate_arm", 10,
                                &arduinoHandler::activate_armCB, this);
    arm_pub = nh.advertise<std_msgs::Int32MultiArray>("/command", 10);
  }
  int map(float inValue, float minOutRange, float maxOutRange) {
    float minInRange = -1.57;
    float maxInRange = 1.57;
    float x = (inValue - minInRange) / (maxInRange - minInRange);
    float result = minOutRange + (maxOutRange - minOutRange) * x;
    return int(result);
  }

  int map_gripper(float inValue, float minOutRange, float maxOutRange) {
    // joint_state goes from 0 to 0.02 we map 0 to upper and 0.02 to lower
    float minInRange = 0.0;
    float maxInRange = 0.02;
    float x = (inValue - minInRange) / (maxInRange - minInRange);
    float result = minOutRange + (maxOutRange - minOutRange) * x;
    return int(result);
  }
  int limit_to_range(int inValue, int minRange, int maxRange) {
    if (inValue > maxRange) {
      inValue = maxRange;
    }
    if (inValue < minRange) {
      inValue = minRange;
    }
  }
  void activate_armCB(const std_msgs::Int32::ConstPtr &act_arm) {
    std_msgs::Int32MultiArray arm_array;
    arm_array.data.clear();
    allow_pub = act_arm->data;
    if (allow_pub) {
      arm_array.data.push_back(4900);
      arm_array.data.push_back(2334);
      arm_array.data.push_back(6844);
      arm_array.data.push_back(5000);
      arm_array.data.push_back(2342); // 2342 // 5000
      arm_array.data.push_back(5150); // 5150 //2000
      arm_array.data.push_back(4000);
      arm_array.data.push_back(-1);

      arm_array.data.push_back(-1);
      arm_array.data.push_back(-1);
      arm_array.data.push_back(-1);
      arm_array.data.push_back(-1);

      arm_array.data.push_back(-1);
      arm_array.data.push_back(-1);
      arm_array.data.push_back(-1);
      arm_array.data.push_back(-1);

      prev_servo_0 = 4900;
      prev_servo_1 = 2300;
      prev_servo_2 = 6800;
      prev_servo_3 = 5000;
      prev_servo_4 = 2000;
      prev_servo_5 = 5150;
      prev_servo_6 = 4000;
      arm_pub.publish(arm_array);
    }
  }

  void jsCB(const sensor_msgs::JointState::ConstPtr &joint_state) {
    std_msgs::Int32MultiArray arm_array;

    arm_array.data.clear();

    curr_0 = joint_state->position[9];
    curr_1 = joint_state->position[8];
    curr_2 = joint_state->position[3];
    curr_3 = joint_state->position[10];
    curr_4 = joint_state->position[11];
    curr_5 = joint_state->position[12];
    curr_6 = joint_state->position[4];

    bool cond_0 = fabs(prev_0 - curr_0) < 0.001;
    bool cond_1 = fabs(prev_1 - curr_1) < 0.001;
    bool cond_2 = fabs(prev_2 - curr_2) < 0.001;
    bool cond_3 = fabs(prev_3 - curr_3) < 0.001;
    bool cond_4 = fabs(prev_4 - curr_4) < 0.001;
    bool cond_5 = fabs(prev_5 - curr_5) < 0.001;
    bool cond_6 = fabs(prev_6 - curr_6) < 0.001;

    if (cond_0 && cond_1 && cond_2 && cond_3 && cond_4 && cond_5 && cond_6) {
      if (allow_pub) {
        arm_array.data.push_back(prev_servo_0 + up_down);
        arm_array.data.push_back(prev_servo_1 + up_down);
        arm_array.data.push_back(prev_servo_2 + up_down);
        arm_array.data.push_back(prev_servo_3 + up_down);
        arm_array.data.push_back(prev_servo_4 + up_down);
        arm_array.data.push_back(prev_servo_5 + up_down);
        arm_array.data.push_back(prev_servo_6 + up_down);
        arm_array.data.push_back(-1);
        arm_array.data.push_back(-1);
        arm_array.data.push_back(-1);
        arm_array.data.push_back(-1);
        arm_array.data.push_back(-1);
        arm_array.data.push_back(-1);
        arm_array.data.push_back(-1);
        arm_array.data.push_back(-1);
        arm_array.data.push_back(-1);
        arm_pub.publish(arm_array);
      }
      up_down = -up_down;
      return;
    }

    int servo_0 = this->map(curr_0, 1600, 8200);
    int servo_1 = this->map(curr_1, 2200, 8200);
    // helper for dpedent link_2
    int helper =
        (8200 - servo_1 + 2300); // link1 5150 -> link2 5900 and reverse

    int servo_2 = helper + 1.3 * (1600 - this->map(curr_2, -1600, 1600));

    int servo_3 = this->map(curr_3, 1800, 8200);
    int servo_4 = this->map(curr_4, 2000, 8300);
    int servo_5 = this->map(curr_5, 2000, 8300);
    int servo_6 = this->map_gripper(curr_6, 4000, 7150);

    if (fabs(servo_2 - helper) > 1600) {
      if (servo_2 > helper) {
        servo_2 = helper + 1600;
      } else {
        servo_2 = helper - 1600;
      }
    }
    // servo_0 = limit_to_range(servo_0, 1600, 8200);
    // servo_2 = limit_to_range(servo_2, 1500, 9900);
    // servo_1 = limit_to_range(servo_1, 2000, 8300);
    // servo_3 = limit_to_range(servo_3, 1800, 8200);
    // servo_4 = limit_to_range(servo_4, 2000, 8300);
    // servo_5 = limit_to_range(servo_5, 2000, 8300);
    // servo_6 = limit_to_range(servo_6, 4000, 7500);

    arm_array.data.push_back(servo_0);
    arm_array.data.push_back(servo_1);
    arm_array.data.push_back(servo_2);
    arm_array.data.push_back(servo_3);
    arm_array.data.push_back(servo_4);
    arm_array.data.push_back(servo_5);
    arm_array.data.push_back(servo_6);
    arm_array.data.push_back(-1);
    arm_array.data.push_back(-1);
    arm_array.data.push_back(-1);
    arm_array.data.push_back(-1);
    arm_array.data.push_back(-1);
    arm_array.data.push_back(-1);
    arm_array.data.push_back(-1);
    arm_array.data.push_back(-1);
    arm_array.data.push_back(-1);
    if (allow_pub) {
      arm_pub.publish(arm_array);
    }

    prev_0 = curr_0;
    prev_1 = curr_1;
    prev_2 = curr_2;
    prev_3 = curr_3;
    prev_4 = curr_4;
    prev_5 = curr_5;
    prev_6 = curr_6;

    prev_servo_0 = servo_0;
    prev_servo_1 = servo_1;
    prev_servo_2 = servo_2;
    prev_servo_3 = servo_3;
    prev_servo_4 = servo_4;
    prev_servo_5 = servo_5;
    prev_servo_6 = servo_6;
  }

protected:
  ros::NodeHandle nh;
  ros::Subscriber js_sub;
  ros::Subscriber activate_sub;
  ros::Publisher arm_pub;
  float curr_0 = 0, curr_1 = 0, curr_2 = 0, curr_3 = 0, curr_4 = 0, curr_5 = 0,
        curr_6 = 0;
  float prev_0 = 0, prev_1 = 0, prev_2 = 0, prev_3 = 0, prev_4 = 0, prev_5 = 0,
        prev_6 = 0;

  float prev_servo_0 = 0, prev_servo_1 = 0, prev_servo_2 = 0, prev_servo_3 = 0,
        prev_servo_4 = 0, prev_servo_5 = 0, prev_servo_6 = 0;

  int up_down = 1;
  int allow_pub = 0;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "driver_to_arduino");
  arduinoHandler handler;
  ros::spin();
  return 0;
}

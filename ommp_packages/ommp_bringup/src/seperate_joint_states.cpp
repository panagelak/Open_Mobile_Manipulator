#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class SeperateJointStates
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher arm_js_pub_, gripper_js_pub_, wheels_js_pub_, kinect_js_pub_;

public:
  explicit SeperateJointStates(ros::NodeHandle nh) : nh_(nh)
  {
    joint_state_sub_ = nh_.subscribe("/joint_states", 10, &SeperateJointStates::joint_statesCB, this);
    arm_js_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states/arm", 10);
    gripper_js_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states/gripper", 10);
    wheels_js_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states/wheels", 10);
    kinect_js_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states/kinect", 10);
  }
  void joint_statesCB(const sensor_msgs::JointState::ConstPtr &joint_state)
  {
    sensor_msgs::JointState arm_js, gripper_js, wheels_js, kinect_js;
    // Arm Position pan->9 lift->8 elbow->3 wrist1->10 wrist2->11 wrist3->12
    // Gripper position finger1 -> 4 finger2 -> 5
    // Wheels bl -> 0 br -> 1 fl-> 6 fr -> 7
    // kinect -> 2

    //ARM
    arm_js.header = joint_state->header;

    arm_js.name.push_back(joint_state->name[9]);
    arm_js.name.push_back(joint_state->name[8]);
    arm_js.name.push_back(joint_state->name[3]);
    arm_js.name.push_back(joint_state->name[10]);
    arm_js.name.push_back(joint_state->name[11]);
    arm_js.name.push_back(joint_state->name[12]);

    arm_js.position.push_back(joint_state->position[9]);
    arm_js.position.push_back(joint_state->position[8]);
    arm_js.position.push_back(joint_state->position[3]);
    arm_js.position.push_back(joint_state->position[10]);
    arm_js.position.push_back(joint_state->position[11]);
    arm_js.position.push_back(joint_state->position[12]);

    arm_js.velocity.push_back(joint_state->velocity[9]);
    arm_js.velocity.push_back(joint_state->velocity[8]);
    arm_js.velocity.push_back(joint_state->velocity[3]);
    arm_js.velocity.push_back(joint_state->velocity[10]);
    arm_js.velocity.push_back(joint_state->velocity[11]);
    arm_js.velocity.push_back(joint_state->velocity[12]);

    arm_js.effort.push_back(joint_state->effort[9]);
    arm_js.effort.push_back(joint_state->effort[8]);
    arm_js.effort.push_back(joint_state->effort[3]);
    arm_js.effort.push_back(joint_state->effort[10]);
    arm_js.effort.push_back(joint_state->effort[11]);
    arm_js.effort.push_back(joint_state->effort[12]);

    //GRIPPER
    gripper_js.header = joint_state->header;

    gripper_js.name.push_back(joint_state->name[4]);
    gripper_js.name.push_back(joint_state->name[5]);

    gripper_js.position.push_back(joint_state->position[4]);
    gripper_js.position.push_back(joint_state->position[5]);


    gripper_js.velocity.push_back(joint_state->velocity[4]);
    gripper_js.velocity.push_back(joint_state->velocity[5]);


    gripper_js.effort.push_back(joint_state->effort[4]);
    gripper_js.effort.push_back(joint_state->effort[5]);

    //WHEELS
    wheels_js.header = joint_state->header;

    wheels_js.name.push_back(joint_state->name[0]);
    wheels_js.name.push_back(joint_state->name[1]);
    wheels_js.name.push_back(joint_state->name[6]);
    wheels_js.name.push_back(joint_state->name[7]);


    wheels_js.position.push_back(joint_state->position[0]);
    wheels_js.position.push_back(joint_state->position[1]);
    wheels_js.position.push_back(joint_state->position[6]);
    wheels_js.position.push_back(joint_state->position[7]);


    wheels_js.velocity.push_back(joint_state->velocity[0]);
    wheels_js.velocity.push_back(joint_state->velocity[1]);
    wheels_js.velocity.push_back(joint_state->velocity[6]);
    wheels_js.velocity.push_back(joint_state->velocity[7]);


    wheels_js.effort.push_back(joint_state->effort[0]);
    wheels_js.effort.push_back(joint_state->effort[1]);
    wheels_js.effort.push_back(joint_state->effort[6]);
    wheels_js.effort.push_back(joint_state->effort[7]);


    //KINECT
    kinect_js.header = joint_state->header;
    kinect_js.name.push_back(joint_state->name[2]);
    kinect_js.position.push_back(joint_state->position[2]);
    kinect_js.velocity.push_back(joint_state->velocity[2]);
    kinect_js.effort.push_back(joint_state->effort[2]);

    // Publish Seperate Joint States
    arm_js_pub_.publish(arm_js);
    gripper_js_pub_.publish(gripper_js);
    wheels_js_pub_.publish(wheels_js);
    kinect_js_pub_.publish(kinect_js);
  }
  ~SeperateJointStates()
  {
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "seperate_joint_states_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  SeperateJointStates handler(nh);
  ros::waitForShutdown();
  return 0;
}
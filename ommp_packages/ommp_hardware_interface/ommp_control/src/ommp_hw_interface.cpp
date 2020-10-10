/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the Ommp
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <ommp_control/ommp_hw_interface.h>

namespace ommp_control
{
OmmpHWInterface::OmmpHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  ROS_INFO_NAMED("ommp_hw_interface", "OmmpHWInterface Ready.");
  //
  wheel_vel_pub = nh_.advertise<std_msgs::Float32MultiArray>("set_vel", 10);
  wheel_enc_sub = nh_.subscribe("/encoder_ticks", 10, &OmmpHWInterface::enc_ticks_CB, this);
}

void OmmpHWInterface::read(ros::Duration &elapsed_time)
{
  // DUMMY PASSTHROUGH
  if (dummy_)
  {
    joint_velocity_[0] = joint_velocity_command_[0];
    joint_position_[0] += joint_velocity_command_[0] * elapsed_time.toSec();
    joint_velocity_[1] = joint_velocity_command_[1];
    joint_position_[1] += joint_velocity_command_[1] * elapsed_time.toSec();
    joint_velocity_[6] = joint_velocity_command_[6];
    joint_position_[6] += joint_velocity_command_[6] * elapsed_time.toSec();
    joint_velocity_[7] = joint_velocity_command_[7];
    joint_position_[7] += joint_velocity_command_[7] * elapsed_time.toSec();

    for (std::size_t joint_id = 2; joint_id < 6; ++joint_id)
      joint_position_[joint_id] = joint_position_command_[joint_id];

    for (std::size_t joint_id = 8; joint_id < num_joints_; ++joint_id)
      joint_position_[joint_id] = joint_position_command_[joint_id];
  }
  // REAL ROBOT - COMMUNICATION THROUGH TOPICS - ROSSERIAL
  else
  {
    this->enc_feedback_to_joint_pos_vel(elapsed_time);
    joint_velocity_[0] = joint_feedback_vel[0];
    joint_position_[0] = joint_feedback_pos[0];
    joint_velocity_[1] = joint_feedback_vel[1];
    joint_position_[1] = joint_feedback_pos[1];
    joint_velocity_[6] = joint_feedback_vel[2];
    joint_position_[6] = joint_feedback_pos[2];
    joint_velocity_[7] = joint_feedback_vel[3];
    joint_position_[7] = joint_feedback_pos[3];

    for (std::size_t joint_id = 2; joint_id < 6; ++joint_id)
      joint_position_[joint_id] = joint_position_command_[joint_id];

    for (std::size_t joint_id = 8; joint_id < num_joints_; ++joint_id)
      joint_position_[joint_id] = joint_position_command_[joint_id];
  }
}

void OmmpHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  /* // Uncomment For real robot To Publish to the Arduino Vel Setpoint Callback
  this -> cmd_to_setpoint();
  // Transform From joint_velocity_command to setpoint
  vel_setpoint_cmd_array.data.push_back(vel_set_cmd[0]);
  vel_setpoint_cmd_array.data.push_back(vel_set_cmd[1]);
  wheel_vel_pub.publish(vel_setpoint_cmd_array);

  // Clear Array
  vel_setpoint_cmd_array.data.clear();
  // Store Prev encoder ticks
  for (int i = 0; i < 4; i++) {
    prev_enc[i] = curr_enc[i];
  }
  //*/
}

void OmmpHWInterface::enforceLimits(ros::Duration &period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}
void OmmpHWInterface::enc_feedback_to_joint_pos_vel(ros::Duration &elapsed_time)
{
  for (int i = 0; i < 4; i++)
  {
    joint_feedback_vel[i] = float((curr_enc[i] - prev_enc[i])) / elapsed_time.toSec();
    joint_feedback_vel[i] *= ENC_TO_VEL_FACTOR;
    joint_feedback_pos[i] = curr_enc[i];
    joint_feedback_pos[i] *= ENC_TO_POS_FACTOR;
  }
}

void OmmpHWInterface::cmd_to_setpoint()
{
  // On the same side they recieve the same command
  vel_set_cmd[0] = joint_velocity_command_[0] * CMD_TO_SET_FACTOR;  // back left -> = left
  vel_set_cmd[1] = joint_velocity_command_[1] * CMD_TO_SET_FACTOR;  // back right -> = right
}

void OmmpHWInterface::enc_ticks_CB(const std_msgs::Int64MultiArray::ConstPtr &enc_msg)
{
  curr_enc[0] = enc_msg->data[2];
  curr_enc[1] = enc_msg->data[3];
  curr_enc[2] = enc_msg->data[0];
  curr_enc[3] = enc_msg->data[1];
}

}  // namespace ommp_control

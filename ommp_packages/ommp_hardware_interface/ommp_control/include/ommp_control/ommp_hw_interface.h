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
           For a more detailed simulation example, see sim_hw_interface.h
*/

#ifndef OMMP_CONTROL__OMMP_HW_INTERFACE_H
#define OMMP_CONTROL__OMMP_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>

// Publish Subscribe Includes
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"

namespace ommp_control
{

// Tranform Factors From encoder to joint_state Universe
static const double ENC_TO_VEL_FACTOR = 0.000679;
static const double ENC_TO_POS_FACTOR = 0.000679;
//cmd msg 0.25 -> 4.5454 -> x = 0.055
static const double CMD_TO_SET_FACTOR = 0.055;


/// \brief Hardware interface for a robot
class OmmpHWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  OmmpHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration &elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration &elapsed_time);

  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration &period);

  // Subscriber wheel encoder Callback
  void enc_ticks_CB(const std_msgs::Int64MultiArray::ConstPtr &enc_msg);
  void cmd_to_setpoint();
  void enc_feedback_to_joint_pos_vel(ros::Duration &elapsed_time);

protected:
  // Publish and Subscriber to from Arduino
  ros::Publisher wheel_vel_pub;
  ros::Subscriber wheel_enc_sub;
  std_msgs::Float32MultiArray vel_setpoint_cmd_array;
  // To hold the encoder ticks and the previous encoder ticks
  // Used to calculate speed
  int curr_enc[4] = {0, 0, 0, 0};
  int prev_enc[4] = {0, 0, 0, 0};
  double joint_feedback_vel[4] = {0., 0., 0., 0.};
  double joint_feedback_pos[4] = {0., 0., 0., 0.};
  double vel_set_cmd[2] = {0., 0.};

};  // class

}  // namespace

#endif

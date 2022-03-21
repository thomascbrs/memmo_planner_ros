// Copyright 2022 University of Edinburgh
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "memmo_teleop/memmo_teleop_joystick.hpp"

namespace memmo_teleop
{
MemmoTeleopJoystick::MemmoTeleopJoystick() : nh_(""), private_nh_("~")
{
  // Init ROS parameter
  // TODO(JaehyunShim): Reset default values.
  // TODO(JaehyunShim): Simplify using struct.
  std::string nodeName = "/memmo_teleop";

  // Velocity limit and step size
  private_nh_.param<double>(nodeName + "/vel_x_limit", vel_x_limit_, 0.5);
  private_nh_.param<double>(nodeName + "/vel_x_step_size", vel_x_step_size_, 0.01);

  private_nh_.param<double>(nodeName + "/vel_y_limit", vel_y_limit_, 0.5);
  private_nh_.param<double>(nodeName + "vel_y_step_size", vel_y_step_size_, 0.01);

  private_nh_.param<double>(nodeName + "/vel_z_limit", vel_z_limit_, 0.5);
  private_nh_.param<double>(nodeName + "/vel_z_step_size", vel_z_step_size_, 0.01);

  private_nh_.param<double>(nodeName + "/vel_roll_limit", vel_roll_limit_, 0.5);
  private_nh_.param<double>(nodeName + "/vel_roll_step_size", vel_roll_step_size_, 0.10);

  private_nh_.param<double>(nodeName + "/vel_pitch_limit", vel_pitch_limit_, 0.5);
  private_nh_.param<double>(nodeName + "/vel_pitch_step_size", vel_pitch_step_size_, 0.10);

  private_nh_.param<double>(nodeName + "/vel_yaw_limit", vel_yaw_limit_, 0.5);
  private_nh_.param<double>(nodeName + "/vel_yaw_step_size", vel_yaw_step_size_, 0.10);

  // Method to control the velocity
  // 0 --> Continusous value from joystick publishing
  // 1 --> Increase step by step the velocity
  private_nh_.param<int>(nodeName + "/method_id", method_id_, 0);

  // Filtering parameters
  private_nh_.param<double>(nodeName + "/publishing_rate", publishing_rate_, 0.5);
  private_nh_.param<double>(nodeName + "/cutoff_frequency", cutoff_frequency_, 0.5);

  // Dead zone sensibility
  private_nh_.param<double>(nodeName + "/dead_zone_x", dead_zone_x_, 0.01);
  private_nh_.param<double>(nodeName + "/dead_zone_y", dead_zone_y_, 0.01);
  private_nh_.param<double>(nodeName + "/dead_zone_yaw", dead_zone_yaw_, 0.01);
  private_nh_.param<double>(nodeName + "/dead_zone_step", dead_zone_step_, 0.99);

  ROS_INFO("Using method : %d", method_id_);

  // Init ROS publisher and subscriber
  // TODO(JaehyunShim): Need more consideration on queue size
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  joy_sub_ = nh_.subscribe("joy", 10, &MemmoTeleopJoystick::joy_callback, this);

  // Create a ROS timer for publishing cmd velocity
  timer_publisher_ = nh_.createTimer(ros::Duration(1.0 / publishing_rate_), std::bind(&MemmoTeleopJoystick::timer_callback, this));

  // cutoff frequency [Hz]
  beta_ = std::exp(-2*M_PI*cutoff_frequency_*(1/publishing_rate_));
  vel_joystick_ = Eigen::Matrix<double, 6, 1>::Zero();
  vel_filtered_ = Eigen::Matrix<double, 6, 1>::Zero();
  first_joy_received_ = false;
}

MemmoTeleopJoystick::~MemmoTeleopJoystick()
{
  // Stop robot
  // send_cmd_vel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void MemmoTeleopJoystick::timer_callback(){
  if (first_joy_received_){
    vel_filtered_ = beta_ * vel_filtered_ + (1-beta_)*vel_joystick_;
    send_cmd_vel(vel_filtered_[0], vel_filtered_[1], vel_filtered_[2], vel_filtered_[3], vel_filtered_[4], vel_filtered_[5]);
  }
}

// Reference: http://wiki.ros.org/joy
void MemmoTeleopJoystick::joy_callback(const sensor_msgs::Joy::ConstPtr & msg)
{
  if (method_id_ == 0){
    update_joystick_continuous(msg);
  }
  if (method_id_ == 1){
    update_joystick_step(msg);
  }
  if (!first_joy_received_){
    first_joy_received_ = true;
  }
}

void MemmoTeleopJoystick::update_joystick_step(const sensor_msgs::Joy::ConstPtr & msg){
  if (msg->axes.at(1) >= dead_zone_step_)
  {  // Move in x direction
    vel_joystick_[0] += vel_x_step_size_;
  }
  if (msg->axes.at(1) <= - dead_zone_step_)
  {  // Move in -x direction
    vel_joystick_[0] -= vel_x_step_size_;
  }
  if (msg->axes.at(0) >= dead_zone_step_)
  {  // Move in y direction
    vel_joystick_[1] += vel_y_step_size_;
  }
  if (msg->axes.at(0) <= - dead_zone_step_)
  {  // Move in -y direction
    vel_joystick_[1] -= vel_y_step_size_;
  }
  if (msg->axes.at(2) >= dead_zone_step_)
  {  // Move in yaw direction
    vel_joystick_[5] += vel_yaw_step_size_;
  }
  if (msg->axes.at(2) <= - dead_zone_step_)
  {  // Move in -yaw direction
    vel_joystick_[5] -= vel_yaw_step_size_;
  }
  vel_joystick_[0] = enforce_vel_limit(vel_joystick_[0], vel_x_limit_);
  vel_joystick_[1] = enforce_vel_limit(vel_joystick_[1], vel_y_limit_);
  vel_joystick_[2] = enforce_vel_limit(vel_joystick_[2], vel_z_limit_);
  vel_joystick_[3] = enforce_vel_limit(vel_joystick_[3], vel_roll_limit_);
  vel_joystick_[4] = enforce_vel_limit(vel_joystick_[4], vel_pitch_limit_);
  vel_joystick_[5] = enforce_vel_limit(vel_joystick_[5], vel_yaw_limit_);
}

void MemmoTeleopJoystick::update_joystick_continuous(const sensor_msgs::Joy::ConstPtr & msg){
  if (msg->axes.at(1) >= dead_zone_x_)
  {  // Move in x direction
    vel_joystick_[0] = msg->axes.at(1) * vel_x_limit_;
  }
  if (msg->axes.at(1) <= -dead_zone_x_)
  { // Move in -x direction
    vel_joystick_[0] = msg->axes.at(1) * vel_x_limit_;
  }
  if (msg->axes.at(0) >= dead_zone_y_)
  {  // Move in y direction
    vel_joystick_[1] = msg->axes.at(0) * vel_y_limit_ ;
  }
  if (msg->axes.at(0) <= -dead_zone_y_)
  { // Move in -y direction
    vel_joystick_[1] = msg->axes.at(0) * vel_y_limit_;
  }
  if (msg->axes.at(2) >= dead_zone_yaw_)
  {  // Move in yaw direction
    vel_joystick_[5] = msg->axes.at(2) * vel_yaw_limit_;
  }
  if (msg->axes.at(2) <= -dead_zone_yaw_)
  { // Move in -yaw direction
    vel_joystick_[5] = msg->axes.at(2) * vel_yaw_limit_;
  }
}

void MemmoTeleopJoystick::print_joyop()
{
  // TODO(JaehyunShim): Rewrite .. Better keyop?
}

// TODO(JaehyunShim): Add smoother

// TODO(JaehyunShim): why has to be reference, not pointer..?
void MemmoTeleopJoystick::send_cmd_vel(double vel_lin_x, double vel_lin_y, double vel_lin_z,
                                      double vel_ang_x, double vel_ang_y, double vel_ang_z)
{
  // Enforce velocity limit
  geometry_msgs::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = vel_lin_x;
  cmd_vel_msg.linear.y = vel_lin_y;
  cmd_vel_msg.linear.z = vel_lin_z;
  cmd_vel_msg.angular.x = vel_ang_x;
  cmd_vel_msg.angular.y = vel_ang_y;
  cmd_vel_msg.angular.z = vel_ang_z;
  cmd_vel_pub_.publish(cmd_vel_msg);

  ROS_INFO("\nvel_lin_x: %.4lf\n", cmd_vel_msg.linear.x);
  ROS_INFO("vel_lin_y: %.4lf\n", cmd_vel_msg.linear.y);
  ROS_INFO("vel_lin_yaw: %.4lf\n", cmd_vel_msg.angular.z);
}

double MemmoTeleopJoystick::enforce_vel_limit(double vel, double limit)
{
  if (std::abs(vel) > limit)
  {
    vel = limit * (vel / std::abs(vel));
  }
  return vel;
}
}  // namespace memmo_teleop

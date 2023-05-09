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

#ifndef MEMMO_TELEOP__MEMMO_TELEOP_KEYBOARD_HPP_
#define MEMMO_TELEOP__MEMMO_TELEOP_KEYBOARD_HPP_

#include <termios.h>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace memmo_teleop {
class MemmoTeleopKeyboard {
public:
  MemmoTeleopKeyboard();
  ~MemmoTeleopKeyboard();

private:
  // Node Handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Linear Velocity
  double lin_vel_x_limit_;
  double lin_vel_x_step_size_;
  double lin_vel_y_limit_;
  double lin_vel_y_step_size_;
  double lin_vel_z_limit_;
  double lin_vel_z_step_size_;

  // Angular Velocity
  double ang_vel_x_limit_;
  double ang_vel_x_step_size_;
  double ang_vel_y_limit_;
  double ang_vel_y_step_size_;
  double ang_vel_z_limit_;
  double ang_vel_z_step_size_;

  // Terminal Settings
  struct termios orig_termios_;
  struct termios new_termios_;

  // ROS Publisher
  ros::Publisher cmd_vel_pub_;

  // Update
  void run();
  void print_keyop();
  // TODO(JaehyunShim): print_vel should be curr vel? or ref vel?
  void send_cmd_vel(double vel_lin_x, double vel_lin_y, double vel_lin_z,
                    double vel_ang_x, double vel_ang_y, double vel_ang_z);
  double enforce_vel_limit(double vel, double limit);
};
} // namespace memmo_teleop
#endif // MEMMO_TELEOP__MEMMO_TELEOP_KEYBOARD_HPP_

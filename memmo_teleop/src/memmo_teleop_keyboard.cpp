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

#include "memmo_teleop/memmo_teleop_keyboard.hpp"

namespace memmo_teleop {
MemmoTeleopKeyboard::MemmoTeleopKeyboard() : nh_(""), private_nh_("~") {
  // Init ROS parameter
  // TODO(JaehyunShim): Reset default values.
  // TODO(JaehyunShim): Simplify using struct.
  private_nh_.param<double>("lin_vel_x_limit", lin_vel_x_limit_, 0.5);
  private_nh_.param<double>("lin_vel_x_step_size", lin_vel_x_step_size_, 0.01);

  private_nh_.param<double>("lin_vel_y_limit", lin_vel_y_limit_, 0.5);
  private_nh_.param<double>("lin_vel_y_step_size", lin_vel_y_step_size_, 0.01);

  private_nh_.param<double>("lin_vel_z_limit", lin_vel_z_limit_, 0.5);
  private_nh_.param<double>("lin_vel_z_step_size", lin_vel_z_step_size_, 0.01);

  private_nh_.param<double>("ang_vel_x_limit", ang_vel_x_limit_, 5.0);
  private_nh_.param<double>("ang_vel_x_step_size", ang_vel_x_step_size_, 0.10);

  private_nh_.param<double>("ang_vel_y_limit", ang_vel_y_limit_, 5.0);
  private_nh_.param<double>("ang_vel_y_step_size", ang_vel_y_step_size_, 0.10);

  private_nh_.param<double>("ang_vel_z_limit", ang_vel_z_limit_, 5.0);
  private_nh_.param<double>("ang_vel_z_step_size", ang_vel_z_step_size_, 0.10);

  // Initialize ROS publisher
  // TODO(JaehyunShim): Need more consideration on queue size
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // Change terminal mode
  tcgetattr(0, &orig_termios_);
  new_termios_ = orig_termios_;
  new_termios_.c_lflag &= ~ICANON;
  new_termios_.c_lflag &= ~ECHO;
  new_termios_.c_cc[VMIN] = 1;
  new_termios_.c_cc[VTIME] = 0;
  new_termios_.c_lflag &= ~ISIG;
  tcsetattr(0, TCSANOW, &new_termios_);

  // Print out keyboard operation
  print_keyop();

  run();
}

MemmoTeleopKeyboard::~MemmoTeleopKeyboard() {
  // Reset terminal mode
  tcsetattr(0, TCSANOW, &orig_termios_);

  // Stop robot
  send_cmd_vel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void MemmoTeleopKeyboard::run() {
  uint8_t count = 0;
  double lin_vel_x_ref = 0.0;
  double lin_vel_y_ref = 0.0;
  double lin_vel_z_ref = 0.0;
  double ang_vel_x_ref = 0.0;
  double ang_vel_y_ref = 0.0;
  double ang_vel_z_ref = 0.0;

  // Handling Ctrl + C (disabled with terminal settings for real-time inputs).
  bool keepRunning=true;

  try {
    while (keepRunning) {
      char ch = std::getchar();
      switch (ch) {
        case 'w':
          lin_vel_x_ref += lin_vel_x_step_size_;
          break;

        case 'x':
          lin_vel_x_ref -= lin_vel_x_step_size_;
          break;

        case 'd':
          lin_vel_y_ref += lin_vel_y_step_size_;
          break;

        case 'a':
          lin_vel_y_ref -= lin_vel_y_step_size_;
          break;

        case 'q':
          ang_vel_z_ref += ang_vel_z_step_size_;
          break;

        case 'e':
          ang_vel_z_ref -= ang_vel_z_step_size_;
          break;

        case 's':
          lin_vel_x_ref = 0.0;
          lin_vel_y_ref = 0.0;
          lin_vel_z_ref = 0.0;
          ang_vel_x_ref = 0.0;
          ang_vel_y_ref = 0.0;
          ang_vel_z_ref = 0.0;
          break;

        case 3: // Ctrl + C
          std::cout << "Ending program." << std::endl;
          keepRunning = false;

        case 27: // Esc key
          std::cout << "Ending program." << std::endl;
          keepRunning = false;

        default:
          break;
      }

      send_cmd_vel(lin_vel_x_ref, lin_vel_y_ref, lin_vel_z_ref, ang_vel_x_ref, ang_vel_y_ref, ang_vel_z_ref);

      // Print keyboard operation every 10 commands
      count += 1;
      if (count == 10) {
        print_keyop();
        count = 0;
      }
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }
}

void MemmoTeleopKeyboard::print_keyop() {
  // TODO(JaehyunShim): Rewrite .. Better keyop?
  ROS_INFO("Key Operation\n");
  ROS_INFO("----------------------------------------\n");
  ROS_INFO("\n");
  ROS_INFO("q w e     y u i  \n");
  ROS_INFO("a s d     h j k  \n");
  ROS_INFO("  x         m    \n");
  ROS_INFO("\n");
  ROS_INFO("w/x: lin_vel_x +-\n");
  ROS_INFO("a/d: lin_vel_y +-\n");
  ROS_INFO("q/e: lin_vel_z +-\n");
  ROS_INFO("u/m: ang_vel_x +-\n");
  ROS_INFO("h/k: ang_vel_y +-\n");
  ROS_INFO("y/i: ang_vel_z +-\n");
  ROS_INFO("\n");
  ROS_INFO("s: Stop\n");
  ROS_INFO("\n");
  ROS_INFO("lin_vel_x limit %.1lf\n", lin_vel_x_limit_);
  ROS_INFO("lin_vel_y limit %.1lf\n", lin_vel_y_limit_);
  ROS_INFO("lin_vel_z limit %.1lf\n", lin_vel_z_limit_);
  ROS_INFO("ang_vel_x limit %.1lf\n", ang_vel_x_limit_);
  ROS_INFO("ang_vel_y limit %.1lf\n", ang_vel_y_limit_);
  ROS_INFO("ang_vel_z limit %.1lf\n", ang_vel_z_limit_);
  ROS_INFO("\n");
}

// TODO(JaehyunShim): Add smoother

// TODO(JaehyunShim): why has to be reference, not pointer..?
void MemmoTeleopKeyboard::send_cmd_vel(double vel_lin_x, double vel_lin_y, double vel_lin_z, double vel_ang_x,
                                       double vel_ang_y, double vel_ang_z) {
  // Enforce velocity limit
  geometry_msgs::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = enforce_vel_limit(vel_lin_x, lin_vel_x_limit_);
  cmd_vel_msg.linear.y = enforce_vel_limit(vel_lin_y, lin_vel_y_limit_);
  cmd_vel_msg.linear.z = enforce_vel_limit(vel_lin_z, lin_vel_z_limit_);
  cmd_vel_msg.angular.x = enforce_vel_limit(vel_ang_x, ang_vel_x_limit_);
  cmd_vel_msg.angular.y = enforce_vel_limit(vel_ang_y, ang_vel_y_limit_);
  cmd_vel_msg.angular.z = enforce_vel_limit(vel_ang_z, ang_vel_z_limit_);
  cmd_vel_pub_.publish(cmd_vel_msg);

  ROS_INFO("vel_lin_x:: %.2lf\n", vel_lin_x);
  ROS_INFO("vel_lin_y: %.2lf\n", vel_lin_y);
  ROS_INFO("vel_lin_z: %.2lf\n", vel_lin_z);
  ROS_INFO("vel_ang_x:: %.2lf\n", vel_ang_x);
  ROS_INFO("vel_ang_y: %.2lf\n", vel_ang_y);
  ROS_INFO("vel_ang_z: %.2lf\n", vel_ang_z);
}

// TODO(JaehyunShim): why has to be reference, not pointer..?
double MemmoTeleopKeyboard::enforce_vel_limit(double vel, double limit) {
  if (std::abs(vel) > limit) {
    vel = limit * (vel / std::abs(vel));
  }
  return vel;
}
}  // namespace memmo_teleop

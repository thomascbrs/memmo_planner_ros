#!/usr/bin/env python3
#
# Copyright 2022 University of Edinburgh
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of  nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory
from visualization_msgs.msg import MarkerArray

import walkgen.FootStepPlanner as FootStepPlanner
import walkgen.GaitManager as GaitManager


class FootstepPlannerNode():

    def __init__(self):
        # Planner output
        self.marker_array = MarkerArray()
        self.swing_traj = MultiDOFJointTrajectory()

        # Planner onoff switch
        self.onoff = False
        # self.onoff = True

        # ROS publishers and subscribers
        self.filtered_hull_marker_array_pub = rospy.Subscriber(
            'filtered_hull_marker_array', MarkerArray, self.filtered_hull_marker_array_callback, queue_size=10)
        self.joint_swing_traj_pub = rospy.Publisher(
            '~joint_swing_traj', MultiDOFJointTrajectory, queue_size=10)

        # ROS timer
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)

    def filtered_hull_marker_array_callback(self, data):
        # Planner code
        # self.swing_traj = walkgen(data)

        # Turn on publishers
        self.onoff = True

    def timer_callback(self, event):
        if self.onoff == True:
            self.joint_swing_traj_pub.publish(self.swing_traj)
        else:
            print("Waiting to receive convex patches.")

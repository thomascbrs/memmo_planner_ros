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

import os

import rospy
from footstep_msgs.msg import FootstepTrajectory
from visualization_msgs.msg import MarkerArray

import walkgen.SurfacePlanner as SurfacePlanner


class SurfacePlannerNode():

    def __init__(self):

        # Surface planner
        self.surface_planner = SurfacePlanner()

        # Planner output
        self.marker_array = MarkerArray()
        self.swing_traj = FootstepTrajectory()

        # Planner onoff switch
        self.onoff = False
        # self.onoff = True

        # ROS publishers and subscribers
        self.hull_marker_array_sub = rospy.Subscriber(
            'plane_seg/hull_marker_array', MarkerArray, self.hull_marker_array_callback, queue_size=10)
        self.filtered_hull_marker_array_pub = rospy.Publisher(
            'filtered_hull_marker_array', MarkerArray, queue_size=10)
        self.foot_swing_traj_pub = rospy.Publisher(
            '~foot_swing_traj', FootstepTrajectory, queue_size=10)

        # ROS timer
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def hull_marker_array_callback(self, data):
        # Compute filtered convex patches and footstep trajectories
        self.marker_array = self.surface_planner.get_marker_array(data)
        self.swing_traj = self.surface_planner.get_swing_traj(data)

        # Turn on publishers
        self.onoff = True

    def timer_callback(self, event):
        if self.onoff == True:
            self.filtered_hull_marker_array_pub.publish(self.marker_array)
            self.foot_swing_traj_pub.publish(self.swing_traj)
        else:
            print("Waiting to receive convex patches.")

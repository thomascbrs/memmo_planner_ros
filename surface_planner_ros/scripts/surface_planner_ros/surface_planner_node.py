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
from footstep_msgs.msg import GaitStatusOnNewPhase
from visualization_msgs.msg import MarkerArray
from walkgen.tools.geometry_utils import reduce_surfaces, remove_overlap_surfaces
from walkgen.WalkgenRosMessageConversion import Sl1mSurfacePublisher, StepManagerInterface
import numpy as np

import walkgen.SurfacePlanner as SurfacePlanner
from time import perf_counter as clock


class SurfacePlannerNode():

    def __init__(self):

        # Surface planner
        filename_config = os.environ["DEVEL_DIR"] + "/memmo_anymal/walkgen/config/params.yaml"
        self.surface_planner = SurfacePlanner(filename_config)

        self.planeseg = self.surface_planner.planeseg  # Use data from planeseg.
        if self.planeseg:
            # Parameters for planeseg postprocessing.
            self._n_points = self.surface_planner._n_points
            self._method_id = self.surface_planner._method_id
            self._poly_size = self.surface_planner._poly_size
            self._min_area = self.surface_planner._min_area
            self._margin = self.surface_planner._margin

        # TODO : Center the position of the surface around the position of the robot.Only for planeseg.
        self._init_surface = self.surface_planner._init_surface
        # Surface filtered
        if self.planeseg:
            self.surfaces_processed = [self.surface_planner._init_surface.vertices]
        else:
            self.surfaces_processed = None
        # Data from stepmanager
        self.q = np.zeros(19)
        self.q[2] = 0.49
        self.b_vref = np.zeros(6)
        self.b_vref[0] = 0.3
        self.gait = None
        self.fsteps = np.zeros((3, 4))

        # Planner onoff switch
        self.onoff = False

        # ROS publishers and subscribers
        if self.planeseg:
            self.hull_marker_array_sub = rospy.Subscriber('plane_seg/hull_marker_array',
                                                        MarkerArray,
                                                        self.hull_marker_array_callback,
                                                        queue_size=10)
        self.fsteps_manager_sub = rospy.Subscriber('walkgen/fstep_manager',
                                                   GaitStatusOnNewPhase,
                                                   self.fsteps_manager_callback,
                                                   queue_size=10)
        self.surfaces_sl1m_pub = Sl1mSurfacePublisher("walkgen/set_surfaces")

        # Stepmanager interface for message conversion.
        self._stepmanager_iface = StepManagerInterface()

        # ROS timer
        self.timer = rospy.Timer(rospy.Duration(0.001), self.timer_callback)

    def hull_marker_array_callback(self, data):
        """ Filter and store incoming convex surfaces from planeseg.
        """
        # Reduce and sort incoming data
        surfaces_reduced = reduce_surfaces(data, margin=self._margin, n_points=self._n_points)

        # Apply proccess to filter and decompose the surfaces to avoid overlap
        self.surfaces_processed = remove_overlap_surfaces(surfaces_reduced,
                                                          polySize=self._poly_size,
                                                          method=self._method_id,
                                                          min_area=self._min_area,
                                                          initial_floor=self._init_surface.vertices)

        # print("\n -----Marker array received-----   \n")

    def fsteps_manager_callback(self, data):
        """ Extract data from foostep manager.
        """
        self.gait, self.fsteps = self._stepmanager_iface.writeFromMessage(data)

        # Turn on planner
        self.onoff = True

    def timer_callback(self, event):
        if self.onoff == True:
            t0 = clock()
            # Start an optimisation
            selected_surfaces = self.surface_planner.run(self.q, self.gait, self.b_vref, self.fsteps,
                                                        self.surfaces_processed)
            t1 = clock()
            print("SL1M optimisation took [ms] : ", 1000 * (t1 - t0))
            # Publish the surfaces.
            t0 = clock()
            self.surfaces_sl1m_pub.publish(0.5, selected_surfaces)
            t1 = clock()
            print("Publisher took [ms] : ", 1000 * (t1 - t0))
            # Turn of switch
            self.onoff = False

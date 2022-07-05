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

import numpy as np
import pinocchio
try:
    from time import perf_counter as clock
except ImportError:
    from time import time as clock
import warnings

from geometry_msgs.msg import Twist
import rospy
import tf
from visualization_msgs.msg import Marker, MarkerArray
from whole_body_state_subscriber_py import WholeBodyStateSubscriber

from footstep_msgs.msg import GaitStatusOnNewPhase, SetSurfaces
from surface_planner_integrated_ros.step_manager_interface import StepManagerInterface
from surface_planner_integrated_ros.surface_planner_interface import SurfacePlannerInterface
from surface_planner_integrated_ros.footstep_visualization import generate_config, generate_footsteps, generate_surfaces

from walkgen_surface_planner import SurfacePlanner
from walkgen_surface_planner.params import SurfacePlannerParams
from walkgen_surface_processing.surface_processing import SurfaceProcessing
from walkgen_surface_processing.params import SurfaceProcessingParams


class SurfacePlannerNode():

    def __init__(self):

        # Define frames
        self.odom_frame = rospy.get_param("~odom_frame")
        self.world_frame = rospy.get_param("~world_frame")
        if self.world_frame == str():
            self.world_frame = self.odom_frame
        self.use_drift_compensation = (self.odom_frame != self.world_frame)
        if not self.use_drift_compensation:
            rospy.loginfo("World and odom frame are identical. Drift compensation is not used.")

        # --------------------------------------------------------------
        # Wait for URDF. Get state of the robot to adapt the surface height.
        while not rospy.is_shutdown():
            if rospy.has_param(rospy.get_param("~robot_description")):
                break
            else:
                rospy.loginfo("URDF not yet available on topic " + rospy.get_param("~robot_description") +
                              " - waiting")
                rospy.sleep(1.)
        urdf_xml = rospy.get_param(rospy.get_param("~robot_description"))
        self.feet_3d_names = rospy.get_param("~3d_feet")
        robot_state_topic = rospy.get_param("~robot_state_topic")
        locked_joint_names = rospy.get_param("~joints_to_be_locked")
        self._model = pinocchio.buildModelFromXML(urdf_xml, pinocchio.JointModelFreeFlyer())
        locked_joint_ids = pinocchio.StdVec_Index()
        for name in locked_joint_names:
            if self._model.existJointName(name):
                locked_joint_ids.append(self._model.getJointId(name))
            else:
                rospy.logwarn("The " + name + " cannot be locked as it doesn't belong to the full model")
        if len(locked_joint_ids) > 0:
            self._model = pinocchio.buildReducedModel(self._model, locked_joint_ids, pinocchio.neutral(self._model))
        self._data = self._model.createData()
        self._ws_sub = WholeBodyStateSubscriber(self._model, robot_state_topic, frame_id=self.odom_frame)
        self._tf_listener = tf.TransformListener()
        self._rot = pinocchio.Quaternion(np.array([0., 0., 0., 1.]))
        self._mMo = pinocchio.SE3(self._rot, np.zeros(3))
        self._oMb = pinocchio.SE3(self._rot, np.zeros(3))
        self._mMb = pinocchio.SE3(self._rot, np.zeros(3))

        print("\n")
        print("Initialize height of the initial surface...\n")
        # Compute the average height of the robot
        counter_height = 0
        height_ = []
        offset_height = -0.85
        q0 = None
        while not rospy.is_shutdown() and counter_height < 20:
            if self._ws_sub.has_new_whole_body_state_message():
                t0, q, v, tau, f = self._ws_sub.get_current_whole_body_state()

                # Update the drift between the odom and world frames
                if self.use_drift_compensation:
                    try:
                        self._mMo.translation[:], (self._rot.x, self._rot.y, self._rot.z,
                                                   self._rot.w) = self._tf_listener.lookupTransform(
                                                       self.world_frame, self.odom_frame, rospy.Time())
                        self._mMo.rotation = self._rot.toRotationMatrix()
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        pass
                # Map the current state from odom to world frame
                self._oMb.translation[:] = q[:3]
                self._rot.x, self._rot.y, self._rot.z, self._rot.w = q[3:7]
                self._oMb.rotation = self._rot.toRotationMatrix()
                q[:7] = pinocchio.SE3ToXYZQUAT(self._mMo.act(self._oMb))

                pinocchio.forwardKinematics(self._model, self._data, q)
                pinocchio.updateFramePlacements(self._model, self._data)
                h = 0
                for name in self.feet_3d_names:
                    frameId = self._model.getFrameId(name)
                    h += self._data.oMf[frameId].translation[2]
                height_.append(h/4)
                counter_height += 1
                q0 = q
                rospy.sleep(0.1)
            elif counter_height == 0:
                rospy.loginfo("Waiting for a new whole body state message.")
                rospy.sleep(1)

        print("Initial configuration in world frame : ", q0[:7])
        self._q = q0[:7]
        print("Average height for initial surface : ", np.mean(height_))
        print("-------------")
        initial_height = np.mean(height_)
        # --------------------------------------------------------------


        ## Surface processing
        self.plane_seg = rospy.get_param("~plane_seg")
        initial_height = 0.85
        self.params_surface_processing = SurfaceProcessingParams()
        self.params_surface_processing.plane_seg = rospy.get_param("~plane_seg")
        self.params_surface_processing.n_points = rospy.get_param("~n_points")
        self.params_surface_processing.method_id = rospy.get_param("~method_id")
        self.params_surface_processing.poly_size = rospy.get_param("~poly_size")
        self.params_surface_processing.min_area = rospy.get_param("~min_area")
        self.params_surface_processing.margin = rospy.get_param("~margin")
        self.params_surface_processing.path = rospy.get_param("~path")
        self.params_surface_processing.stl = rospy.get_param("~stl")
        self.surface_processing = SurfaceProcessing(initial_height, self.params_surface_processing)

        ## Surface planner
        # Waiting for MPC-Walkgen parameters
        while not rospy.is_shutdown():
            if rospy.has_param(rospy.get_param("~N_uds")):
                break
            else:
                rospy.loginfo("Waiting for MPC-Walkgen")
                rospy.sleep(1)

        # Surface Planner parameters independant from MPC-Walkgen-Caracal
        self.params_surface_planner = SurfacePlannerParams()
        self.params_surface_planner.N_phase = rospy.get_param("~N_phase")
        self.params_surface_planner.com = rospy.get_param("~com")
        self.params_surface_planner.typeGait = rospy.get_param(rospy.get_param("~typeGait"))
        self.params_surface_planner.N_ss = rospy.get_param(rospy.get_param("~N_ss"))
        self.params_surface_planner.N_ds = rospy.get_param(rospy.get_param("~N_ds"))
        self.params_surface_planner.N_uss = rospy.get_param(rospy.get_param("~N_uss"))
        self.params_surface_planner.N_uds = rospy.get_param(rospy.get_param("~N_uds"))
        self.params_surface_planner.dt = rospy.get_param(rospy.get_param("~dt"))
        self.params_surface_planner.N_phase_return = rospy.get_param(rospy.get_param("~N_phase_return"))
        if self.params_surface_planner.N_phase_return > self.params_surface_planner.N_phase:
            warnings.warn("More phases in the MPC horizon than planned bby the MIP. The last surface selected will be used multiple times.")
        self.params_surface_planner.fitsize_x = rospy.get_param("~fitsize_x")
        self.params_surface_planner.fitsize_y = rospy.get_param("~fitsize_y")
        self.params_surface_planner.fitlength = rospy.get_param("~fitlength")
        self.params_surface_planner.recompute_slope = rospy.get_param("~recompute_slope")
        self.surface_planner = SurfacePlanner(self.params_surface_planner)

        # For plane_publisher
        # all_surfaces = surface_detector.extract_surfaces()
        # self.surface_planner.set_surfaces(all_surfaces)

        # Surface filtered
        self.surfaces_processed = None
        self.new_surfaces = False
        self.surface_planner_switch = False
        self.footstep_manager_switch = False
        self.gait = None
        self.footsteps = np.zeros((3, 4))
        self.q_filter = np.zeros(7)
        self.cmd_vel = np.array([0., 0., 0., 0., 0., 0.])

        # Interfaces for message conversion
        self.footstep_manager_interface = StepManagerInterface()
        self.surface_planner_interface = SurfacePlannerInterface()

        # ROS publishers and subscribers
        if self.plane_seg:
            hull_marker_array_topic_name = "/plane_seg/hull_marker_array"
        else:
            hull_marker_array_topic_name = "/plane_publisher/hull_marker_array"
        self.hull_marker_array_sub = rospy.Subscriber(
            hull_marker_array_topic_name , MarkerArray, self.hull_marker_array_callback, queue_size=10)
        self.marker_array_pub = rospy.Publisher(
            "~visualization_marker_array", MarkerArray, queue_size=10)
        self.footstep_manager_sub = rospy.Subscriber(
            "/walkgen/footstep_manager", GaitStatusOnNewPhase, self.footstep_manager_callback, queue_size=10)
        self.surface_planner_pub = rospy.Publisher("/walkgen/surface_planner", SetSurfaces, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)

        # ROS timer
        self.timer = rospy.Timer(rospy.Duration(0.001), self.timer_callback)

    def cmd_vel_callback(self, msg):
        self.cmd_vel[0] = msg.linear.x
        self.cmd_vel[1] = msg.linear.y
        self.cmd_vel[5] = msg.angular.z

    def hull_marker_array_callback(self, msg):
        """ Filter and store incoming convex surfaces from plane_seg.
        """
        print("\n -----Marker array received-----   \n")
        if self.plane_seg:
            self.surfaces_processed = self.surface_processing.run(self._q[:3], msg)
            surfaces = [np.array(value).T for key,value in self.surfaces_processed.items()]
            msg = generate_surfaces(surfaces, frame_id=self.world_frame)
        else:
            self.surfaces_processed = msg

        # Publish surfaces
        self.marker_array_pub.publish(msg)

        # Turn on surface planner switch
        self.new_surfaces = True
        self.surface_planner_switch = True

    def footstep_manager_callback(self, msg):
        """ Extract data from foostep manager.
        """
        self.gait, self.footsteps, q_filter = self.step_manager_interface.writeFromMessage(msg)
        self.q_filter[:3] = q_filter[:3]
        self.q_filter[3:7] = pinocchio.Quaternion(pinocchio.rpy.rpyToMatrix(q_filter[3:])).coeffs()

        # Turn on footstep manager switch
        self.footstep_manager_switch = True

    def timer_callback(self, event):
        if self.surface_planner_switch and self.footstep_manager_switch:
            t_init, q, v, tau, f = self._ws_sub.get_current_whole_body_state()

            # Update the drift between the odom and world frames
            if self.use_drift_compensation:
                try:
                    self._mMo.translation[:], (self._rot.x, self._rot.y, self._rot.z,
                                                self._rot.w) = self._tf_listener.lookupTransform(
                                                    self.world_frame, self.odom_frame, rospy.Time())
                    self._mMo.rotation = self._rot.toRotationMatrix()
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    raise ArithmeticError("Problem with tf transform")

            # Map the current state from odom to world frame
            self._oMb.translation[:] = q[:3]
            self._rot.x, self._rot.y, self._rot.z, self._rot.w = q[3:7]
            self._oMb.rotation = self._rot.toRotationMatrix()
            q[:7] = pinocchio.SE3ToXYZQUAT(self._mMo.act(self._oMb))
            self._q = q[:7]

            t0 = clock()

            # Start optimization
            if self.new_surfaces:
                self.surface_planner.set_surfaces(self.surfaces_processed)  # Update surfaces
                self.new_surfaces = False
            selected_surfaces = self.surface_planner.run(self.q_filter, self.gait, self.cmd_vel, self.footsteps)

            if self.surface_planner.pb_data.success:
                t1 = clock()
                print("SL1M optimization took [ms] : ", 1000 * (t1 - t0))

                t0 = clock()
                # Publish config
                msg = generate_config(self.surface_planner.configs, lifetime = self.surface_planner._step_duration, frame_id=self.world_frame)
                self.marker_array_pub.publish(msg)

                # Publish footsteps
                msg = generate_footsteps(self.surface_planner.pb_data.all_feet_pos, lifetime = self.surface_planner._step_duration, frame_id=self.world_frame)
                self.marker_array_pub.publish(msg)

                # Publish surfaces
                surfaces = [np.array(value).T for key,value in self.surface_planner.all_surfaces.items()]
                msg = generate_surfaces(surfaces, frame_id=self.world_frame)
                self.marker_array_pub.publish(msg)

                t1 = clock()
                print("Publisher for visualization took [ms] : ", 1000 * (t1 - t0))

                # Publish surfaces
                t0 = clock()
                msg = self.surface_planner_interface.writeToMessage(0.5, selected_surfaces)
                self.surface_planner_pub.publish(msg)

                t1 = clock()
                print("Publisher took [ms] : ", 1000 * (t1 - t0))

            # Turn off footstep manager switch
            self.footstep_manager_switch = False

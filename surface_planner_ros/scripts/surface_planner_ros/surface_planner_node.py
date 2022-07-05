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
try:
    from time import perf_counter as clock
except ImportError:
    from time import time as clock
from time import sleep
import pinocchio
import warnings

from geometry_msgs.msg import Twist
import rospy
import tf
from visualization_msgs.msg import Marker, MarkerArray
from whole_body_state_subscriber_py import WholeBodyStateSubscriber

from footstep_msgs.msg import GaitStatusOnNewPhase, SetSurfaces
from surface_planner_ros.step_manager_interface import StepManagerInterface
from surface_planner_ros.surface_planner_interface import SurfacePlannerInterface
from surface_planner_ros.world_visualization import WorldVisualization

from walkgen_surface_planner import SurfacePlanner
from walkgen_surface_planner.params import SurfacePlannerParams
from walkgen_surface_processing.surface_detector import SurfaceDetector
from walkgen_surface_processing.surface_processing import SurfaceProcessing
from walkgen_surface_processing.params import SurfaceProcessingParams


class SurfacePlannerNode():

    def __init__(self):
        # ------ Get state of the robot to adapt the surface height.
        # Wait for URDF
        while not rospy.is_shutdown():
            if rospy.has_param(rospy.get_param("~robot_description")):
                break
            else:
                rospy.loginfo("URDF not yet available on topic " + rospy.get_param("~robot_description") +
                              " - waiting")
                sleep(1)

        # Define frames
        self._odomFrame = rospy.get_param("~odom_frame")
        self._worldFrame = rospy.get_param("~world_frame")
        if self._worldFrame == str():
            self._worldFrame = self._odomFrame
        self._useDriftCompensation = (self._odomFrame != self._worldFrame)
        if not self._useDriftCompensation:
            rospy.loginfo("Map and odom frame are identical: Not using drift compensation.")
        urdfXml = rospy.get_param(rospy.get_param("~robot_description"))
        self._feet3DNames = rospy.get_param("~3d_feet")
        robotStateTopic = rospy.get_param("~robot_state_topic")
        lockedJointNames = rospy.get_param("~joints_to_be_locked")
        self._model = pinocchio.buildModelFromXML(urdfXml, pinocchio.JointModelFreeFlyer())
        lockedJointIds = pinocchio.StdVec_Index()
        for name in lockedJointNames:
            if self._model.existJointName(name):
                lockedJointIds.append(self._model.getJointId(name))
            else:
                rospy.logwarn("The " + name + " cannot be locked as it doesn't belong to the full model")
        if len(lockedJointIds) > 0:
            self._model = pinocchio.buildReducedModel(self._model, lockedJointIds, pinocchio.neutral(self._model))
        self._data = self._model.createData()
        self._ws_sub = WholeBodyStateSubscriber(self._model, robotStateTopic, frame_id=self._odomFrame)
        self._tf_listener = tf.TransformListener()
        self._rot = pinocchio.Quaternion(np.array([0., 0., 0., 1.]))
        self._mMo = pinocchio.SE3(self._rot, np.zeros(3))
        self._oMb = pinocchio.SE3(self._rot, np.zeros(3))
        self._mMb = pinocchio.SE3(self._rot, np.zeros(3))

        print("\n Initialize height of the initial surface...")
        # Compute the average height of the robot
        counter_height = 0
        height_ = []
        offset_height = -0.85
        q0 = None
        while not rospy.is_shutdown() and counter_height < 20:
            if self._ws_sub.has_new_whole_body_state_message():
                t0, q, v, tau, f = self._ws_sub.get_current_whole_body_state()

                # Update the drift between the odom and world frames
                if self._useDriftCompensation:
                    try:
                        self._mMo.translation[:], (self._rot.x, self._rot.y, self._rot.z,
                                                   self._rot.w) = self._tf_listener.lookupTransform(
                                                       self._worldFrame, self._odomFrame, rospy.Time())
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
                for name in self._feet3DNames:
                    frameId = self._model.getFrameId(name)
                    h += self._data.oMf[frameId].translation[2]
                height_.append(h/4)
                counter_height += 1
                q0 = q
                sleep(0.1)
            elif counter_height == 0:
                rospy.loginfo("Waiting for a new whole body state message.")
                sleep(1)

        print("Initial configuration in world frame : ", q0[:7])
        self._q = q0[:7]
        print("Average height for initial surface : ", np.mean(height_))
        print("-------------")
        initial_height = np.mean(height_)

        self._visualization = rospy.get_param("~visualization")
        self._footstep_manager_topic = rospy.get_param("~footstep_manager_topic")
        self._surface_planner_topic = rospy.get_param("~surface_planner_topic")
        self._plane_seg_topic = rospy.get_param("~plane_seg_topic")

        ## Surface processing
        # Post-processing & environment parameters
        self.params_surface_processing = SurfaceProcessingParams()

        self.params_surface_processing.plane_seg = rospy.get_param("~plane_seg")
        self.params_surface_processing.n_points = rospy.get_param("~n_points")
        self.params_surface_processing.method_id = rospy.get_param("~method_id")
        self.params_surface_processing.poly_size = rospy.get_param("~poly_size")
        self.params_surface_processing.min_area = rospy.get_param("~min_area")
        self.params_surface_processing.margin = rospy.get_param("~margin")
        self.params_surface_processing.path = rospy.get_param("~path")
        self.params_surface_processing.stl = rospy.get_param("~stl")
        self.plane_seg = self.params_surface_processing.plane_seg  # Use data from plane_seg.

        # Surface processing
        self.surface_processing = SurfaceProcessing(initial_height= initial_height, params = self.params_surface_processing)
        self._firstSetSurfaces = False

        ## Surface planner
        # Surface Planner parameters independant from MPC-Walkgen-Caracal
        self.params_surface_planner = SurfacePlannerParams()
        self.params_surface_planner.N_phase = rospy.get_param("~N_phase")
        self.params_surface_planner.com = rospy.get_param("~com")

        # Waiting for MPC-Walkgen parameters
        while not rospy.is_shutdown():
            if rospy.has_param(rospy.get_param("~N_uds")):
                break
            else:
                rospy.loginfo("Waiting for MPC-Walkgen")
                sleep(1)

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

        # Surface planner
        self.surface_planner = SurfacePlanner(self.params_surface_planner)

        if not self.plane_seg:
            self._firstSetSurfaces = True # Always available using plane_seg
            # Extract surfaces from URDF file.
            # surface_detector = SurfaceDetector(self._params.path + self._params.urdf, self._params.margin, q0=q0[:7], initial_height=initial_height)
            translation = np.zeros(3)
            translation[:2] = self._q[:2]
            translation[-1] = initial_height
            R_ =  pinocchio.Quaternion(self._q[3:]).toRotationMatrix()
            surface_detector = SurfaceDetector(self.params_surface_processing.path + self.params_surface_processing.stl, R_, translation, self.params_surface_processing.margin , "environment_")

            all_surfaces = surface_detector.extract_surfaces()
            self.surface_planner.set_surfaces(all_surfaces)

        # Visualization tools
        if self._visualization:
            self.world_visualization = WorldVisualization()
            if not self.plane_seg:  # Publish URDF environment
                print("Publishing world...")
                # self.world_visualization = WalkgenVisualizationPublisher()
                sleep(1.)  # Not working otherwise

                worldMesh = self.params_surface_processing.path + self.params_surface_processing.stl
                worldPose = self._q
                worldPose[2] = initial_height

                self.marker_pub = rospy.Publisher(
                    "surface_planner/visualization_marker", Marker, queue_size=10)
                self.marker_array_pub = rospy.Publisher(
                    "surface_planner/visualization_marker_array", MarkerArray, queue_size=10)

                sleep(1.)  # Not working otherwise

                msg = self.world_visualization.generate_world(worldMesh, worldPose, frame_id=self._worldFrame)
                self.marker_pub.publish(msg)

                surfaces = [np.array(value).T for value in all_surfaces.values()]
                msg = self.world_visualization.generate_surfaces(surfaces, frame_id=self._worldFrame)
                self.marker_array_pub.publish(msg)

        # Surface filtered
        self.surfaces_processed = None
        self._newSurfaces = False

        # cmd_vel subscriber
        self.cmd_vel = np.array([0., 0., 0., 0., 0., 0.])
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)

        self.gait = None
        self.fsteps = np.zeros((3, 4))
        self.q_filter = np.zeros(7)

        # Planner onoff switch
        self.onoff = False

        # ROS publishers and subscribers
        if self.plane_seg:
            self.hull_marker_array_sub = rospy.Subscriber(
                self._plane_seg_topic, MarkerArray, self.hull_marker_array_callback, queue_size=10)
        self.fsteps_manager_sub = rospy.Subscriber(
            self._footstep_manager_topic, GaitStatusOnNewPhase, self.footstep_manager_callback, queue_size=10)

        self.surface_planner_interface = SurfacePlannerInterface()
        self.surface_planner_pub = rospy.Publisher(
            self._surface_planner_topic, SetSurfaces, queue_size=10)

        # Stepmanager interface for message conversion.
        self._step_manager_interface = StepManagerInterface()

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
        self.surfaces_processed = self.surface_processing.run(self._q[:3], msg)
        self._newSurfaces = True
        self._firstSetSurfaces = True
        surfaces = [np.array(value).T for key,value in self.surfaces_processed.items()]
        msg = self.world_visualization.generate_surfaces(surfaces, frame_id=self._worldFrame)
        self.marker_array_pub.publish(msg)

    def footstep_manager_callback(self, msg):
        """ Extract data from foostep manager.
        """
        self.gait, self.fsteps, q_filter = self._step_manager_interface.writeFromMessage(msg)
        self.q_filter[:3] = q_filter[:3]
        self.q_filter[3:7] = pinocchio.Quaternion(pinocchio.rpy.rpyToMatrix(q_filter[3:])).coeffs()

        # Turn on planner
        self.onoff = True

    def timer_callback(self, event):
        if self.onoff and self._firstSetSurfaces:
            t_init, q, v, tau, f = self._ws_sub.get_current_whole_body_state()

            # Update the drift between the odom and world frames
            if self._useDriftCompensation:
                try:
                    self._mMo.translation[:], (self._rot.x, self._rot.y, self._rot.z,
                                                self._rot.w) = self._tf_listener.lookupTransform(
                                                    self._worldFrame, self._odomFrame, rospy.Time())
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

            # Start an optimisation
            if self._newSurfaces:
                self.surface_planner.set_surfaces(self.surfaces_processed)  # Update surfaces
                self._newSurfaces = False
            selected_surfaces = self.surface_planner.run(self.q_filter, self.gait, self.cmd_vel, self.fsteps)

            if self.surface_planner.pb_data.success:
                t1 = clock()
                print("SL1M optimisation took [ms] : ", 1000 * (t1 - t0))
                if self._visualization:
                    t0 = clock()
                    # Publish world config
                    msg = self.world_visualization.generate_config(self.surface_planner.configs, lifetime = self.surface_planner._step_duration, frame_id=self._worldFrame)
                    self.marker_array_pub.publish(msg)

                    # Publish world footsteps
                    msg = self.world_visualization.generate_footsteps(self.surface_planner.pb_data.all_feet_pos, lifetime = self.surface_planner._step_duration, frame_id=self._worldFrame)
                    self.marker_array_pub.publish(msg)

                    # Publish world surfaces
                    surfaces = [np.array(value).T for key,value in self.surface_planner.all_surfaces.items()]
                    msg = self.world_visualization.generate_surfaces(surfaces, frame_id=self._worldFrame)
                    self.marker_array_pub.publish(msg)

                    t1 = clock()
                    print("Publisher for visualization took [ms] : ", 1000 * (t1 - t0))

                # Publish surfaces
                t0 = clock()
                msg = self.surface_planner_interface.writeToMessage(0.5, selected_surfaces)
                self.surface_planner_pub.publish(msg)

                t1 = clock()
                print("Publisher took [ms] : ", 1000 * (t1 - t0))

            # Turn off planner
            self.onoff = False

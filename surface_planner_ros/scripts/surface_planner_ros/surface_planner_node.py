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
from crocoddyl_ros import WholeBodyStateRosSubscriber

from footstep_msgs.msg import GaitStatusOnNewPhase, SetSurfaces
from footstep_msgs.srv import Clearmap
from surface_planner_ros.step_manager_interface import StepManagerInterface
from surface_planner_ros.surface_planner_interface import SurfacePlannerInterface
from surface_planner_ros.world_visualization import WorldVisualization
from surface_planner_ros.Logger import Logger
from surface_planner_ros.elevation_map_interface import ElevationMapInterface, DECOMPO_ALGO

from walkgen_surface_planner import SurfacePlanner
from walkgen_surface_planner.params import SurfacePlannerParams
from walkgen_surface_processing.surface_detector import SurfaceDetector
from walkgen_surface_processing.surface_loader import SurfaceLoader
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
        # self._ws_sub = WholeBodyStateSubscriber(self._model, robot_state_topic, frame_id=self.odom_frame)
        self._ws_sub = WholeBodyStateRosSubscriber(self._model, robot_state_topic, frame=self.odom_frame)
        self._tf_listener = tf.TransformListener()
        self._rot = pinocchio.Quaternion(np.array([0., 0., 0., 1.]))
        self._mMo = pinocchio.SE3(self._rot, np.zeros(3))
        self._oMb = pinocchio.SE3(self._rot, np.zeros(3))
        self._mMb = pinocchio.SE3(self._rot, np.zeros(3))

        if rospy.has_param(rospy.get_param("~initial_config")):
            self._q = np.array(rospy.get_param(rospy.get_param("~initial_config")))
            initial_height = rospy.get_param(rospy.get_param("~initial_floor_height"))
        else:
            # Get initial config & update server parameter.
            self._q, initial_height = self.set_initial_configuration()

        self._visualization = rospy.get_param("~visualization")
        self._footstep_manager_topic = rospy.get_param("~footstep_manager_topic")
        self._surface_planner_topic = rospy.get_param("~surface_planner_topic")
        self._plane_seg_topic = rospy.get_param("~plane_seg_topic")

        # Records timings
        self._RECORDING = rospy.get_param("~recording")
        if self._RECORDING:
            folder_path = rospy.get_param("~folder_path")
            self._logger = Logger(folder_path)

        ########
        ## Surface processing
        self.params_surface_processing = SurfaceProcessingParams()
        self.params_surface_processing.plane_seg = rospy.get_param("~plane_seg")
        self.params_surface_processing.extract_mehtodId = rospy.get_param("~extract_methodId")
        self.params_surface_processing.n_points = rospy.get_param("~n_points")
        self.params_surface_processing.method_id = rospy.get_param("~method_id")
        self.params_surface_processing.poly_size = rospy.get_param("~poly_size")
        self.params_surface_processing.min_area = rospy.get_param("~min_area")
        self.params_surface_processing.margin_inner = rospy.get_param("~margin_inner")
        self.params_surface_processing.margin_outer = rospy.get_param("~margin_outer")
        self.params_surface_processing.path = rospy.get_param("~path")
        self.params_surface_processing.stl = rospy.get_param("~stl")
        self.params_surface_processing.offset_z = rospy.get_param("~offset_z")
        self.plane_seg = self.params_surface_processing.plane_seg  # Use data from plane_seg.
        self.surface_processing = SurfaceProcessing(initial_height=initial_height,
                                                    params=self.params_surface_processing)
        self.first_set_surfaces = False

        ########
        ## Surface Planner parameters independant from MPC-Walkgen-Caracal
        self.params_surface_planner = SurfacePlannerParams()
        self.params_surface_planner.N_phase_return = rospy.get_param("~N_phase_return")
        self.params_surface_planner.horizon = rospy.get_param("~horizon")
        self.params_surface_planner.com = rospy.get_param("~com")
        self.params_surface_planner.contact_names = self.feet_3d_names
        self.params_surface_planner.shoulder_offsets = rospy.get_param("~shoulder_offsets")
        # Heightmap parameters
        self.params_surface_planner.fitsize_x = rospy.get_param("~fitsize_x")
        self.params_surface_planner.fitsize_y = rospy.get_param("~fitsize_y")
        self.params_surface_planner.fitlength = rospy.get_param("~fitlength")
        self.params_surface_planner.recompute_slope = rospy.get_param("~recompute_slope")
        self.surface_planner = SurfacePlanner(self.params_surface_planner, RECORDING=self._RECORDING)


        if not self.plane_seg:
            self.first_set_surfaces = True  # Always available using plane_seg
            # Extract surfaces from URDF file.
            # surface_detector = SurfaceDetector(self._params.path + self._params.urdf, self._params.margin, q0=q0[:7], initial_height=initial_height)
            translation = np.zeros(3)
            # translation[:2] = self._q[:2]
            translation[-1] = initial_height
            # R_ = pinocchio.Quaternion(self._q[3:]).toRotationMatrix()
            R_ = np.identity(3)
            if self.params_surface_processing.extract_mehtodId == 0:
                # Single file .stl
                surface_detector = SurfaceDetector(
                    self.params_surface_processing.path + self.params_surface_processing.stl, R_, translation,
                    self.params_surface_processing.margin_inner, "environment_", self.params_surface_processing.offset_z)
            else:
                # Folder containing multiple .stl files.
                surface_detector = SurfaceLoader(
                    self.params_surface_processing.path + self.params_surface_processing.stl, R_, translation,
                    "environment_", self.params_surface_processing, True)
            all_surfaces = surface_detector.extract_surfaces()

        # Visualization tools
        if self._visualization:
            self.world_visualization = WorldVisualization()
            self.marker_pub = rospy.Publisher("surface_planner/visualization_marker", Marker, queue_size=10)
            self.marker_array_pub = rospy.Publisher("surface_planner/visualization_marker_array",
                                                    MarkerArray,
                                                    queue_size=10)
            if not self.plane_seg:  # Publish URDF environment
                print("Publishing world...")
                # self.world_visualization = WalkgenVisualizationPublisher()
                # rospy.sleep(1.)  # Not working otherwise

                worldMesh = self.params_surface_processing.path + self.params_surface_processing.stl
                worldPose = self._q
                worldPose[2] = initial_height

                rospy.sleep(1.)  # Not working otherwise

                msg = self.world_visualization.generate_world(worldMesh, worldPose, frame_id=self.world_frame)
                if self.params_surface_processing.extract_mehtodId == 0 : # Publish only when using 1 single stl file.
                    self.marker_pub.publish(msg)
                    print("World published.")

                surfaces = [np.array(value).T for value in all_surfaces.values()]
                msg = self.world_visualization.generate_surfaces(surfaces, frame_id=self.world_frame)
                self.marker_array_pub.publish(msg)

        print("Visualization loop finished.")
        if not self.plane_seg:
            self.surface_planner.set_surfaces(all_surfaces)

        # Surface filtered
        self.surfaces_processed = None
        self.new_surfaces = False
        # Gait matrix.
        self.gait = None
        # List of timings associated with the gait matrix.
        self.gait_timings = None
        self.footsteps = np.zeros((3, 4))
        self.q_filter = np.zeros(7)

        # Others
        self.cmd_vel = np.array([0., 0., 0., 0., 0., 0.])  # Command velocity
        self.planner_switch = False  # Planner onoff switch

        # Interfaces for message conversion
        self.footstep_manager_interface = StepManagerInterface()
        self.surface_planner_interface = SurfacePlannerInterface()

        # ROS publishers and subscribers
        if self.plane_seg:
            self.hull_marker_array_sub = rospy.Subscriber(self._plane_seg_topic,
                                                          MarkerArray,
                                                          self.hull_marker_array_callback,
                                                          queue_size=10)
            
            self._elevation_map_topic = "/convex_plane_decomposition_ros/planar_terrain"
            self.elevation_map_sub = rospy.Subscriber(self._elevation_map_topic,
                                                          MarkerArray,
                                                          self.elevation_map_callback,
                                                          queue_size=10)
        self.footstep_manager_sub = rospy.Subscriber(self._footstep_manager_topic,
                                                     GaitStatusOnNewPhase,
                                                     self.footstep_manager_callback,
                                                     queue_size=10)
        self.surface_planner_pub = rospy.Publisher(self._surface_planner_topic, SetSurfaces, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)

        # Service to clear the surfaces under a certain height.
        self._clearmap_srv = rospy.Service("walkgen/clearmap", Clearmap, self.clearmapService)

        # ROS timer
        self.timer = rospy.Timer(rospy.Duration(0.005), self.timer_callback)

    def cmd_vel_callback(self, msg):
        self.cmd_vel[0] = msg.linear.x
        self.cmd_vel[1] = msg.linear.y
        self.cmd_vel[5] = msg.angular.z

    def hull_marker_array_callback(self, msg):
        """ Filter and store incoming convex surfaces from plane_seg.
        """
        print("\n -----Marker array received-----   \n")
        t0 = clock()
        self.surfaces_processed = self.surface_processing.run(self._q[:3], msg)
        t1 = clock()
        if self._RECORDING:
            self._logger._profiler["timing_processing"].append(t1 - t0)
            self._logger._profiler["processing_number"].append(len(self.surfaces_processed.values()))
        print("Process hull marker [ms] : ", 1000 * (t1 - t0))
        self.new_surfaces = True
        self.first_set_surfaces = True
        if self._visualization:
            surfaces = [np.array(value).T for key, value in self.surfaces_processed.items()]
            msg = self.world_visualization.generate_surfaces(surfaces, frame_id=self.world_frame)
            self.marker_array_pub.publish(msg)
    
    def elevation_map_callback(self, msg):
        """ Filter and store incoming planes which are non-convex coming
        from elevation_map_cupy.
        """
        t0 =clock()
        self.surfaces_processed = self.map_interface.process(msg)
        t1 = clock()
        print("Process elevation map planes [ms] : ", 1000*(t1 - t0))

        self.new_surfaces = True
        self.first_set_surfaces = True
        if self._visualization:
            surfaces = [np.array(value).T for key, value in self.surfaces_processed.items()]
            msg = self.world_visualization.generate_surfaces(surfaces, frame_id=self.world_frame)
            self.marker_array_pub.publish(msg)

    def footstep_manager_callback(self, msg):
        """ Extract data from foostep manager.
        """
        self.gait, self.gait_timings, self.footsteps, q_filter = self.footstep_manager_interface.writeFromMessage(msg)
        self.q_filter[:3] = q_filter[:3]
        self.q_filter[3:7] = pinocchio.Quaternion(pinocchio.rpy.rpyToMatrix(q_filter[3:])).coeffs()

        # Turn on planner
        self.planner_switch = True

    def clearmapService(self, req):
        """ Service function to remove the ground floor surfaces from the surface post-processing.
        """
        print("\n Clearmap service received.")
        self.surface_processing.set_clearmap(req.clearmap)
        self.surface_processing.set_offset_clearmap(req.offset)
        print("Clearmap parameter : ", self.surface_processing._clearmap)
        print("Offset parameter   : ", self.surface_processing._offsets_clearmap)
        print("\n")

        return True

    def timer_callback(self, event):
        if self.planner_switch and self.first_set_surfaces:
            self._t0, q, v, tau, _, _, f, _ = self._ws_sub.get_state()

            # Update the drift between the odom and world frames
            if self.use_drift_compensation:
                try:
                    self._mMo.translation[:], (self._rot.x, self._rot.y, self._rot.z,
                                               self._rot.w) = self._tf_listener.lookupTransform(
                                                   self.world_frame, self.odom_frame, rospy.Time())
                    self._mMo.rotation = self._rot.toRotationMatrix()
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    if self._isSetup == False:
                        return
                    else:
                        pass
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
            selected_surfaces = self.surface_planner.run(self.q_filter, self.gait, self.gait_timings, self.cmd_vel,
                                                         self.footsteps)

            if self._RECORDING:
                self._logger.update_logger(self.surface_planner.get_profiler())
                self._logger.write_data()
                # Reset processing
                self._logger.reset_data()

            if self.surface_planner.pb_data.success:
                t1 = clock()
                print("Run function took [ms] : ", 1000 * (t1 - t0))
                if self._visualization:
                    t0 = clock()
                    # Publish world config
                    msg = self.world_visualization.generate_config(self.surface_planner.configs,
                                                                   lifetime=self.surface_planner._step_duration,
                                                                   frame_id=self.world_frame)
                    self.marker_array_pub.publish(msg)

                    # Publish world footsteps
                    msg = self.world_visualization.generate_footsteps(self.surface_planner.pb_data.all_feet_pos,
                                                                      lifetime=self.surface_planner._step_duration,
                                                                      frame_id=self.world_frame)
                    self.marker_array_pub.publish(msg)

                    # Publish world surfaces
                    surfaces = [np.array(value).T for key, value in self.surface_planner.all_surfaces.items()]
                    msg = self.world_visualization.generate_surfaces(surfaces, frame_id=self.world_frame)
                    self.marker_array_pub.publish(msg)

                    t1 = clock()
                    print("Publisher for visualization took [ms] : ", 1000 * (t1 - t0))

                # Publish surfaces
                t0 = clock()
                msg = self.surface_planner_interface.writeToMessage(0.5, selected_surfaces)
                self.surface_planner_pub.publish(msg)

                t1 = clock()
                print("Publisher took [ms] : ", 1000 * (t1 - t0))
                print("\n --- \n")

            # Turn off planner
            self.planner_switch = False

    def set_initial_configuration(self):
        """Set the initial configuration in world frame to the server param. Usefull for SL1M to be restarted on the fly.
        Set initial height of the ground. 
        """
        # Compute the average height of the robot
        counter_height = 0
        height_ = []
        q0 = None
        while not rospy.is_shutdown() and counter_height < 10:
            if self._ws_sub.has_new_msg():
                self._t0, q, v, tau, _, _, f, _ = self._ws_sub.get_state()

                # Update the drift between the odom and world frames
                if self.use_drift_compensation:
                    try:
                        self._mMo.translation[:], (self._rot.x, self._rot.y, self._rot.z,
                                                   self._rot.w) = self._tf_listener.lookupTransform(
                                                       self.world_frame, self.odom_frame, rospy.Time())
                        self._mMo.rotation = self._rot.toRotationMatrix()
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        if self._isSetup == False:
                            continue
                        else:
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
                height_.append(h / 4)
                counter_height += 1
                q0 = q
                rospy.sleep(0.01)
            # elif counter_height == 0:
            #     rospy.loginfo("Waiting for a new whole body state message.")
            #     # rospy.sleep(1)

        print("Setting initial parameters on rosparam server.")
        rospy.set_param(rospy.get_param("~initial_config"), q0[:7].tolist())
        rospy.set_param(rospy.get_param("~initial_floor_height"), float(np.mean(height_)))
        return q0[:7], np.mean(height_)

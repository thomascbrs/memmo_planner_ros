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

# import rospy
# from footstep_msgs.msg import FootstepTrajectory
# from visualization_msgs.msg import MarkerArray


import numpy as np
import pinocchio

import rospy
import tf
from visualization_msgs.msg import Marker, MarkerArray
from whole_body_state_subscriber_py import WholeBodyStateSubscriber

from plane_publisher.plane_visualization import generate_world, generate_surfaces
from walkgen_surface_processing.surface_detector import SurfaceDetector
from walkgen_surface_processing.surface_processing import SurfaceProcessing
from walkgen_surface_processing.params import SurfaceProcessingParams


class PlanePublisherNode():

    def __init__(self):

        # Define frames
        self.odom_frame = rospy.get_param("~odom_frame")
        self.world_frame = rospy.get_param("~world_frame")
        if self.world_frame == str():
            self.world_frame = self.odom_frame
        self.use_drift_compensation = (self.odom_frame != self.world_frame)
        if not self.use_drift_compensation:
            rospy.loginfo("World and odom frame are identical. Drift compensation is not used.")

        # Wait for URDF. Get the state of the robot to adapt the surface height
        while not rospy.is_shutdown():
            if rospy.has_param(rospy.get_param("~robot_description")):
                break
            else:
                rospy.loginfo("URDF is not available yet")
                rospy.loginfo("Waiting for the topic \"" + rospy.get_param("~robot_description") + "\"")
                rospy.sleep(1.)
        urdf_xml = rospy.get_param(rospy.get_param("~robot_description"))
        self.feet_3d_names = rospy.get_param("~3d_feet")
        robot_state_topic = rospy.get_param("~robot_state_topic")
        locked_joint_names = rospy.get_param("~joints_to_be_locked")
        self.model = pinocchio.buildModelFromXML(urdf_xml, pinocchio.JointModelFreeFlyer())
        locked_joint_ids = pinocchio.StdVec_Index()
        for name in locked_joint_names:
            if self.model.existJointName(name):
                locked_joint_ids.append(self.model.getJointId(name))
            else:
                rospy.logwarn("The " + name + " cannot be locked as it doesn't belong to the full model")
        if len(locked_joint_ids) > 0:
            self.model = pinocchio.buildReducedModel(self.model, locked_joint_ids, pinocchio.neutral(self.model))
        self.data = self.model.createData()
        self.whole_body_state_sub = WholeBodyStateSubscriber(self.model, robot_state_topic, frame_id=self.odom_frame)
        self.tf_listener = tf.TransformListener()
        self.rot = pinocchio.Quaternion(np.array([0., 0., 0., 1.]))
        self.mMo = pinocchio.SE3(self.rot, np.zeros(3))
        self.oMb = pinocchio.SE3(self.rot, np.zeros(3))
        self.mMb = pinocchio.SE3(self.rot, np.zeros(3))

        # Compute the average height of the robot
        print("----------------------------------------------")
        print("Preparing to publish the world\n")
        print("Initializing the height of the initial surface")
        counter_height = 0
        height_ = []
        offset_height = -0.85
        q0 = None
        while not rospy.is_shutdown() and counter_height < 20:
            if self.whole_body_state_sub.has_new_whole_body_state_message():
                t0, q, v, tau, f = self.whole_body_state_sub.get_current_whole_body_state()

                # Update the drift between the odom and world frames
                if self.use_drift_compensation:
                    try:
                        self.mMo.translation[:], (self.rot.x, self.rot.y, self.rot.z,
                                                   self.rot.w) = self.tf_listener.lookupTransform(
                                                       self.world_frame, self.odom_frame, rospy.Time())
                        self.mMo.rotation = self.rot.toRotationMatrix()
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        pass
                # Map the current state from odom to world frame
                self.oMb.translation[:] = q[:3]
                self.rot.x, self.rot.y, self.rot.z, self.rot.w = q[3:7]
                self.oMb.rotation = self.rot.toRotationMatrix()
                q[:7] = pinocchio.SE3ToXYZQUAT(self.mMo.act(self.oMb))

                pinocchio.forwardKinematics(self.model, self.data, q)
                pinocchio.updateFramePlacements(self.model, self.data)
                h = 0
                for name in self.feet_3d_names:
                    frameId = self.model.getFrameId(name)
                    h += self.data.oMf[frameId].translation[2]
                height_.append(h/4)
                counter_height += 1
                q0 = q
                rospy.sleep(0.1)
            elif counter_height == 0:
                rospy.loginfo("Waiting for a new whole body state message.")
                rospy.sleep(1.)

        print("Initial configuration in the world frame : ", q0[:7])
        self._q = q0[:7]
        print("Average height for initial surface : ", round(np.mean(height_), 4))
        print("----------------------------------------------")
        initial_height = np.mean(height_)
        self.init_height = initial_height

        # Extracting surfaces
        self.margin = rospy.get_param("~margin")
        self.path = rospy.get_param("~path")
        self.stl = rospy.get_param("~stl")
        self.use_urdf_surfaces = rospy.get_param("~use_urdf_surfaces")
        if self.use_urdf_surfaces:
            # Extract surfaces from URDF file
            surface_detector = SurfaceDetector(
                self.path + self.urdf,
                self.margin,
                q0=q0[:7],
                initial_height=initial_height)
        else:
            # Extract surfaces from STL file
            translation = np.zeros(3)
            translation[:2] = self._q[:2]
            translation[-1] = initial_height
            R_ =  pinocchio.Quaternion(self._q[3:]).toRotationMatrix()
            surface_detector = SurfaceDetector(
                self.path + self.stl,
                R_,
                translation,
                self.margin,
                "environment_")

        self.surfaces_extracted = surface_detector.extract_surfaces()

        # ROS publishers and subscribers
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        self.marker_array_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)

        # ROS timer
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback)

        rospy.sleep(1.)

    def timer_callback(self, event):
        # Publish world
        worldMesh = self.path + self.stl
        worldPose = self._q
        worldPose[2] = self.init_height
        msg = generate_world(worldMesh, worldPose, frame_id=self.world_frame)
        self.marker_pub.publish(msg)
        print("Published world")

        # Publish surfaces
        surfaces = [np.array(value).T for value in self.surfaces_extracted.values()]
        msg = generate_surfaces(surfaces, frame_id=self.world_frame)
        self.marker_array_pub.publish(msg)
        print("Published surfaces")

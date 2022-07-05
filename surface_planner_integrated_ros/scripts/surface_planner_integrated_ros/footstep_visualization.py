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
import pinocchio as pin

from geometry_msgs.msg import Point
import rospy
from visualization_msgs.msg import Marker, MarkerArray


def generate_footsteps(all_feet_pos, lifetime=0., frame_id="map"):
    """ Generate the foostep optimised by sl1m.

    Args:
        - all_feet_pos (list) : List containing the footstep optimised by the MIP.
        - lifetime (float): Duration in [s].
        - frame_id (str): Frame.
    """
    msg = MarkerArray()
    color = [[1., 0., 0., 1.], [0., 0., 1., 1.],
                [0., 1., 0., 1.], [0., 1., 1., 1.]]
    counter = 0
    for foot_id, all_foot_pos in enumerate(all_feet_pos):
        for foot_pos in all_foot_pos:
            if foot_pos is not None:
                marker_x = Marker()
                _set_header(
                    marker_x, id=counter, frame_id=frame_id, ns="fsteps", lifetime=lifetime)
                _set_pose(marker_x, [foot_pos[0], foot_pos[1], foot_pos[2]], [
                                0., 0., 0., 1.])
                _set_color(marker_x, color[foot_id])
                _set_scale(marker_x, 3*[0.06])
                marker_x.type = marker_x.SPHERE
                marker_x.action = marker_x.ADD
                marker_x.frame_locked = True
                msg.markers.append(marker_x)
                counter += 1

    return msg

def generate_config(configs, lifetime=0., frame_id="map"):
    """ Generate the configuration of each phase of contact of the MIP (arrow on x-axis and arrow on y-axis).

    Args:
        - configs (list): List of config (array x7 Positon and Orientation)
        - lifetime (float): Duration in [s].
        - frame_id (str): Frame.
    """
    msg = MarkerArray()
    rpy = np.array([0, 0, np.pi/2])
    mat_y = pin.rpy.rpyToMatrix(rpy)
    for id, config in enumerate(configs):
        marker_x = Marker()
        color = [1., 0., 0., 1.]
        pose = np.array(config)
        _set_header(marker_x, id=id, frame_id=frame_id,
                            ns="arrow_x", lifetime=lifetime)
        _set_pose(marker_x, pose[:3], pose[3:])
        _set_color(marker_x, color)
        _set_scale(marker_x, [0.1, 0.01, 0.01])
        marker_x.type = marker_x.ARROW
        marker_x.action = marker_x.ADD
        marker_x.frame_locked = True
        msg.markers.append(marker_x)

        marker_y = Marker()
        color = [0., 1., 0., 1.]
        quat = pin.Quaternion(
            np.dot(mat_y, pin.Quaternion(config[3:]).toRotationMatrix()))
        _set_header(marker_y, id=id+50, frame_id=frame_id,
                            ns="arrow_x", lifetime=lifetime)
        _set_pose(marker_y, pose[:3], quat.coeffs())
        _set_color(marker_y, color)
        _set_scale(marker_y, [0.1, 0.01, 0.01])
        marker_y.type = marker_y.ARROW
        marker_y.action = marker_y.ADD
        marker_y.frame_locked = True
        msg.markers.append(marker_y)

    return msg

def generate_surfaces(surfaces, lifetime=0, frame_id="map"):
    """ Generate the surfaces computed by the MIP.
    Args:
        - surfaces (list): List of array 3xn vertices positions:
                    array([[x0, x1, ... , xn],
                            [y0, y1, ... , yn],
                            [z0, z1, ... , zn]])
        - lifetime (float): Duration in [s].
        - frame_id (str): Frame.
    """
    if surfaces[0].shape[0] != 3:
        raise ArithmeticError("Vertices should be an array of size 3xn")

    msg = MarkerArray()
    color = [1., 0., 0., 1.]
    for id, vertices in enumerate(surfaces):
        marker = Marker()
        _set_header(marker, id=id, frame_id=frame_id,
                            ns="hull", lifetime=lifetime)
        _set_pose(marker, [0., 0., 0.], [0., 0., 0., 1.])
        _set_color(marker, color)
        _set_scale(marker, 3*[0.03])
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.frame_locked = True

        # Add points [P0,P1,P1,P2...]
        for k in range(vertices.shape[1] - 1):
            marker.points.append(_point(vertices[:, k]))
            marker.points.append(_point(vertices[:, k+1]))

        # Add end line
        marker.points.append(_point(vertices[:, 0]))
        marker.points.append(_point(vertices[:, -1]))

        # Add marker to markerArray
        msg.markers.append(marker)

    return msg

def _set_header(marker, id, frame_id, ns, lifetime):
    """ Set the header parameters for marker type msg.
    """
    marker.id = id
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.lifetime = rospy.Duration(lifetime)

def _set_pose(marker, pose, orientation):
    """ Set the pose (x7 position, orientation) for marker type msg.
    """
    marker.pose.position.x = pose[0]
    marker.pose.position.y = pose[1]
    marker.pose.position.z = pose[2]
    marker.pose.orientation.x = orientation[0]
    marker.pose.orientation.y = orientation[1]
    marker.pose.orientation.z = orientation[2]
    marker.pose.orientation.w = orientation[3]

def _set_color(marker, color):
    """ Set the color for marker type msg.
    """
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]

def _set_scale(marker, scale):
    """ Set the scale for marker type msg.
    """
    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]

def _point(position):
    """ Return geometry_msgs Point type, from 3x array/list.
    """
    return Point(x=position[0], y=position[1], z=position[2])

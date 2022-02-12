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

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as a3
import numpy as np
import os
import pickle
from time import perf_counter as clock
import visvalingamwyatt as vw

from example_robot_data.robots_loader import ANYmalLoader
import pinocchio as pin
from sl1m.generic_solver import solve_MIP
from sl1m.tools import plot_tools as plot
from sl1m.problem_definition import Problem

from surface_planner.utils import order, compute_inner_inequalities, compute_inner_vertices


class Slim():

    def __init__(self):

        # Load example marker array
        file_path = os.path.dirname(os.path.abspath(__file__)) + "/example_marker_array.pickle"
        with open(file_path, 'rb') as object_file:
            self.array_markers = pickle.load(object_file)

        # Plot surfaces given by MarkerArray topic
        fig = plt.figure()
        ax = plt.axes(projection='3d')

        self.h_init = -0.06
        self.init_vertices = [[0.4, -1, self.h_init],[0.4,1.,self.h_init],[-0.5,1.,self.h_init],[-0.5,-1,self.h_init]]
        X = [pt[0] for pt in self.init_vertices]
        Y = [pt[1] for pt in self.init_vertices]
        Z = [pt[2] for pt in self.init_vertices]
        ax.scatter(X,Y,Z) # Plot vertices
        X.append(self.init_vertices[0][0])
        Y.append(self.init_vertices[0][1])
        Z.append(self.init_vertices[0][2])
        ax.plot3D(X,Y,Z) # Plot edges

        for marker in self.array_markers.markers:
            points_ = marker.points
            X = [pt.x for pt in points_]
            Y = [pt.y for pt in points_]
            Z = [pt.z for pt in points_]
            ax.scatter(X,Y,Z)

            X,Y,Z = [],[],[]

            for k in range(int(len(points_)/2)):

                X.append(points_[2*k].x)
                Y.append(points_[2*k].y)
                Z.append(points_[2*k].z)
                X.append(points_[2*k+1].x)
                Y.append(points_[2*k+1].y)
                Z.append(points_[2*k+1].z)
                ax.plot3D(X,Y,Z)

        self.h_init = -0.06
        self.init_vertices = [[0.65, -1, self.h_init],[0.65,1.,self.h_init],[-0.65,1.,self.h_init],[-0.65,-1,self.h_init]]

        # SL1M initialization
        paths = [os.environ["INSTALL_HPP_DIR"] + "/anymal-rbprm/com_inequalities/feet_quasi_flat/anymal_",
                os.environ["INSTALL_HPP_DIR"] + "/anymal-rbprm/relative_effector_positions/anymal_"]
        suffix_com="_effector_frame_quasi_static_reduced.obj"
        limbs = ['LFleg', 'RFleg', 'LHleg', 'RHleg']
        others = ['LF_ADAPTER_TO_FOOT', 'RF_ADAPTER_TO_FOOT', 'LH_ADAPTER_TO_FOOT', 'RH_ADAPTER_TO_FOOT']
        suffix_feet="_reduced.obj"

        pb = Problem(limb_names=limbs, other_names=others, constraint_paths=paths, suffix_com= suffix_com, suffix_feet= suffix_feet)

        # Plot 3D
        # fig = plt.figure(figsize=(8, 6))
        # ax = plt.axes(projection='3d')
        # all_surfaces = surface_processing(array_markers)
        # for sf in all_surfaces :
        #     plot.plot_surface(sf,ax=ax)

        ANYmalLoader.free_flyer = True
        anymal = ANYmalLoader().robot
        # Initialisation of model quantities
        pin.centerOfMass(anymal.model, anymal.data, anymal.q0)
        pin.updateFramePlacements(anymal.model, anymal.data)
        pin.crba(anymal.model, anymal.data, anymal.q0)

        indexes = ['LF_FOOT', 'RF_FOOT', 'LH_FOOT', 'RH_FOOT']
        self.offsets_feet = np.zeros((3,4))
        for i,idx in enumerate(indexes) :
            Id = anymal.model.getFrameId(idx)
            self.offsets_feet[:,i] = anymal.data.oMf[Id].translation

        t0 = clock()

        # Walking gait
        gait = np.array([[0.,1.,1.,1.],
                        [1.,0.,1.,1.],
                        [1.,1.,0.,1.],
                        [1.,1.,1.,0.]])
        self.T_gait = 2.
        n_gait = 4  # Number of different phases in the gait
        N_phase = 12  # Number of phases
        bvref = np.array([0.1,0.,0.])  # Reference velocity
        initial_config = np.array([0.,0.3,0.,0.,0.,0.,1.])  # Initial config

        # Initialisation of model quantities
        q = anymal.q0.copy()
        q[:3] = initial_config[:3]
        pin.centerOfMass(anymal.model, anymal.data, q)
        pin.updateFramePlacements(anymal.model, anymal.data)
        pin.crba(anymal.model, anymal.data, q)
        current_contacts = np.zeros((3,4))
        for i,idx in enumerate(indexes) :
            Id = anymal.model.getFrameId(idx)
            current_contacts[:,i] = anymal.data.oMf[Id].translation

        current_contacts[2,:] = self.h_init

        configs = []
        configs.append(initial_config.tolist())

        for i in range(1,N_phase):
            config = np.zeros(7)
            config[:3] = bvref*(self.T_gait/n_gait)*i + initial_config[:3]
            rpy = np.array([0.,0.,0.])
            config[3:] = pin.Quaternion(pin.rpy.rpyToMatrix(rpy)).coeffs()
            configs.append(config.tolist())

        R = [pin.XYZQUATToSE3(np.array(config)).rotation for config in configs]

        initial_contacts = [np.array(current_contacts[:, i].tolist()) for i in range(4)]

        all_surfaces = self.surface_processing(self.array_markers)
        surfaces, empty_list = self.get_potential_surfaces(configs, gait, all_surfaces)

        # Without CoM optimization
        # costs = {"effector_positions": [1.0, effector_positions]}
        # pb.generate_problem(R, surfaces, gait, initial_contacts, c0=None,  com=False)

        # With CoM optimization
        pb.generate_problem(R, surfaces, gait, initial_contacts, configs[0][:3],  com=True)

        # Generate costs
        com_positions = self.compute_com_positions(configs, pb)
        effector_positions = self.compute_effector_positions(configs, bvref,pb)
        costs = { "effector_positions": [10.0, effector_positions] ,"coms_3D": [0.1,com_positions]}

        pb_data = solve_MIP(pb, costs=costs,  com=True)

        t1 = clock()
        print("Run took ", 1000. * (t1-t0))
        # Plot 3D
        fig = plt.figure(figsize=(8, 6))
        ax = plt.axes(projection='3d')
        all_surfaces = self.surface_processing(self.array_markers)
        for sf in all_surfaces :
            plot.plot_surface(sf,ax=ax)
        plot.plot_planner_result(pb_data.all_feet_pos, coms=pb_data.coms, ax=ax, show=True)


    def surface_processing(self, markerArray):
        ''' Process the surfaces list from markerArray data type.
        Returns a list of surfaces, defined by the vertices.
        Args :
        - markerArray
        '''
        surface_list = []
        surface_list.append(np.array(self.init_vertices).T)

        for id,marker in enumerate(self.array_markers.markers) :
            # Marker structure :
            # [Pt1,Pt2,Pt2,Pt3,Pt3,Pt4, ... , Ptn-1, Ptn, Pt1, Ptn] !Warning order at the end
            if id != 6 :
                pts = [[pt.x,pt.y,pt.z] for pt in marker.points] # List not sorted, with duplicates
                vertices = order(np.array(pts)) # Sorted, no duplicates

                # Reduce the number of point using Visvalingam’s algorithm
                simplifier = vw.Simplifier(vertices)
                vertices_vw = simplifier.simplify(number=6)

                margin = 0.01
                ineq_inner, ineq_inner_vect, normal = compute_inner_inequalities(vertices_vw, margin)
                vertices_inner = compute_inner_vertices(vertices_vw, ineq_inner, ineq_inner_vect )
                vertices_inner = order(vertices_inner) # If margin create intersection, need to be sorted

                surface_list.append(vertices_inner.T)

        return surface_list

    def get_potential_surfaces(self, configs, gait, all_surfaces):
        """
        Get the rotation matrix and surface condidates for each configuration in configs
        :param configs: a list of successive configurations of the robot
        :param gait: a gait matrix
        :return: a list of surface candidates
        """
        surfaces_list = []
        empty_list = False
        for id, config in enumerate(configs):
            foot_surfaces = []
            stance_feet = np.nonzero(gait[id % len(gait)] == 1)[0]
            previous_swing_feet = np.nonzero(gait[(id-1) % len(gait)] == 0)[0]
            moving_feet = stance_feet[np.in1d(stance_feet, previous_swing_feet, assume_unique=True)]

            foot_surfaces.append(all_surfaces)
            surfaces_list.append(foot_surfaces)

        return surfaces_list, empty_list

    def compute_com_positions(self, configs,pb):
        """
        Compute the com positions
        :param configs the list of configurations
        """
        com_positions = []
        for phase in pb.phaseData:
            com = configs[phase.id][:3]
            # com[2] += 0.5
            com_positions.append(configs[phase.id][:3])

        return com_positions

    def compute_effector_positions(self, configs, bvref, pb):
        """
        Compute the desired effector positions
        :param configs the list of configurations
        :param bvref, Array (x3) the desired velocity in base frame
        """
        # TODO: Divide by number of phases in gait
        t_stance = self.T_gait / 2
        effector_positions = np.zeros((4, pb.n_phases, 2))

        for phase in pb.phaseData:
            for foot in phase.moving:
                rpy = pin.rpy.matrixToRpy(pin.Quaternion(np.array(configs[phase.id][3:7])).toRotationMatrix())
                yaw = rpy[2]  # Get yaw for the predicted configuration
                shoulders = np.zeros(2)
                # Compute heuristic position in horizontal frame
                rpy[2] = 0.  # Yaw = 0. in horizontal frame
                Rp = pin.rpy.rpyToMatrix(rpy)[:2, :2]
                heuristic = 0.5 * t_stance * Rp @ bvref[:2] + Rp @ self.offsets_feet[:2,foot]

                # Compute heuristic in world frame, rotation
                shoulders[0] = heuristic[0] * np.cos(yaw) - heuristic[1] * np.sin(yaw)
                shoulders[1] = heuristic[0] * np.sin(yaw) + heuristic[1] * np.cos(yaw)
                effector_positions[foot][phase.id] = np.array(configs[phase.id][:2] + shoulders)

        return effector_positions

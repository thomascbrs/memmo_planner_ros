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
import rospy
try:
    from time import perf_counter as clock
except ImportError:
    from time import time as clock

import pinocchio as pin
from scipy.spatial import ConvexHull
import visvalingamwyatt as vw
from enum import Enum

# Decomposition algorithm
import walkgen_surface_processing.tools.Tess2 as Tess2
import walkgen_surface_processing.tools.Snoeyink_Keil as Snoeyink_Keil
import walkgen_surface_processing.tools.Bayazit as Bayazit
from walkgen_surface_processing import Bayazit as BayazitCpp
from walkgen_surface_processing import Point
import walkgen_surface_processing.tools.EarCut as EarCut

from walkgen_surface_processing.tools.geometry_utils import process_tess_results

# class Point():
#     def __init__(self, point):
#         self.x = point[0]
#         self.y = point[1]


class DECOMPO_ALGO(Enum):
    Tess2 = 0
    Bayazit = 1
    EarCut = 2
    SnoeyinkKeil = 3


class Algorithm():
    """ Common interface for 4 different types of computational geometry algorithms.
    """
    def __init__(self, type=DECOMPO_ALGO.Tess2, polySize=10):
        """
        Args :
            - type (Enum DECOMPO_ALGO) : the type of algorithm used.
        """
        self.type = type
        self.tess = Tess2.hxGeomAlgo_Tess2()
        self.snoe_keil = Snoeyink_Keil.hxGeomAlgo_SnoeyinkKeil()
        # self.bayazit = Bayazit.hxGeomAlgo_Bayazit()
        self.bayazit = BayazitCpp()
        self.earcut = EarCut.hxGeomAlgo_EarCut()

        # Maximum polysize for Tessa glorithm.
        self.polySize = polySize

    def check_polygon(self, polygon):
        """ Check the polygon type.
        """
        if type(polygon) != list:
            raise AttributeError("polygon should be a list of points.")
        try:
            x = polygon[0].x
        except:
            raise AttributeError("polygon should containg Point type with .x and .y attributes.")

    def check_holes(self, holes):
        if holes != None:
            if type(holes) != list:
                raise AttributeError("holes should be a list of polygons.")
            for hole in holes:
                self.check_polygon(hole)

    def decomposePoly(self, polygon, holes=None):
        """ Decompose a polygon into convex surfaces. A polygon is described as a list 
        of points : [ [pt.x, pt.y] ... ].

        Args:
            - polygon (list) : The outer contour.
            - holes (None of list of polygon) : Holes inside the polygon. 
        """
        # Verify incoming args.
        self.check_polygon(polygon)
        self.check_holes(holes)

        if holes is None:
            # No holes inside the polygon.
            if self.type == DECOMPO_ALGO.Tess2:
                outer_contour = self.get_contour(polygon)
                res = self.tess.tesselate([outer_contour], polySize=self.polySize)
                return self.process_tess_res(res, self.polySize)

            if self.type == DECOMPO_ALGO.Bayazit:
                return self.bayazit.decomposePoly(polygon)
            if self.type == DECOMPO_ALGO.SnoeyinkKeil:
                return self.snoe_keil.decomposePoly(polygon)
            if self.type == DECOMPO_ALGO.EarCut:
                res = self.earcut.triangulate(polygon)
                return self.earcut.polygonize(res)
        else:
            # Holes inside the polygon.
            if self.type == DECOMPO_ALGO.Tess2:
                outer_contour = self.get_contour(polygon)
                inner_contour = [self.get_contour(hole) for hole in holes]
                res = self.tess.difference([outer_contour], inner_contour, polySize=self.polySize)
                return self.process_tess_res(res, self.polySize)

            if self.type == DECOMPO_ALGO.EarCut:
                res = self.earcut.triangulate(polygon, holes)
                return self.earcut.polygonize(res)

            else:
                # Remove the holes by getting a single contour merging outer and inner holes.
                t0 = clock()
                polygon_reshaped = [Point(point[0], point[1]) for point in self.earcut.removeHoles(polygon, holes)]
                t1 = clock()
                # print("Remove holes [ms] : ", 1000 * (t1 - t0))
                if self.type == DECOMPO_ALGO.Bayazit:
                    return self.bayazit.decomposePoly(polygon_reshaped)
                if self.type == DECOMPO_ALGO.SnoeyinkKeil:
                    return self.snoe_keil.decomposePoly(polygon_reshaped)

    def get_contour(self, polygon):
        """ Format the polygon for Tess algorithm.
        Args:
            - polygon (list): List of Points().
        Returns :
            - list: The list containing the vertices of the contour line such as
                            [x0,y0,x1,y1, ... , xn,yn].
        """
        contour = []  # Contour representation [x0,y0,x1,y1, ... , xn,yn]
        for point in polygon:
            contour.append(point.x)
            contour.append(point.y)
        return contour

    def process_tess_res(self, res, poly_numbers):
        """ Get the resutls in a list of polygons from tess polygon decomposition.

        Args:
            - res (Tess object): Result object from Tess library.
            - poly_numbers (int): Maximum number of vertices for the polygon decomposition.

        Returns:
            - List or None:  List of (array 2xn) surfaces defined using the vertices positions:
                        array([[x0, x1, ... , xn],
                                [y0, y1, ... , yn]]).
        """
        if res.elementCount == 0:
            return []  # empty list

        vertices_Ids = []
        polygons = []
        for i in range(res.elementCount):
            list_Id = [id for id in res.elements[poly_numbers * i:poly_numbers * (i + 1)] if id != -1]
            vertices_Ids.append(list_Id)

        # Get 2D surfaces
        for vertices_Id in vertices_Ids:
            polygon = []
            for k, Id in enumerate(vertices_Id):
                polygon.append(Point(res.vertices[2 * Id], res.vertices[2 * Id + 1]))
            polygons.append(polygon)
        return polygons


class ElevationMapInterface():
    def __init__(self, threshold=0.001, polySize=10, DECOMPO_ALGO=DECOMPO_ALGO.Tess2, convexHoles=False):
        self.tess = Tess2.hxGeomAlgo_Tess2()
        self.snoe_keil = Snoeyink_Keil.hxGeomAlgo_SnoeyinkKeil()
        self.bayazit = Bayazit.hxGeomAlgo_Bayazit()
        self.earcut = EarCut.hxGeomAlgo_EarCut()

        self.DECOMPO_ALGO = DECOMPO_ALGO
        self.algorithm = Algorithm(DECOMPO_ALGO, polySize)
        self.threshold = threshold
        self.polySize = polySize
        self.convexHoles = convexHoles

        self.offset_z = 0.036

        # Store a second algorithm with another method to decompose the surfaces
        # Ensure most of the decomposition will succeed.
        self.has_security = True
        self.security_algorithm = None
        self.security_type = None
        print("\n")
        print("Post-process filtering :")
        print("========================")
        print("\n")
        print("Using decomposition with : ", self.DECOMPO_ALGO.name + " algorithm.")
        if DECOMPO_ALGO in [DECOMPO_ALGO.EarCut, DECOMPO_ALGO.Bayazit, DECOMPO_ALGO.SnoeyinkKeil]:
            self.security_type = DECOMPO_ALGO.Tess2
            self.security_algorithm = Algorithm(self.security_type, polySize)
            print("Using security decomposition with : " + self.security_type.name + " algorithm.")
        else:
            self.security_type = DECOMPO_ALGO.Bayazit
            self.security_algorithm = Algorithm(self.security_type, polySize)
            print("Using security decomposition with : " + self.security_type.name + " algorithm.")
        if not self.has_security:
            print("No security algorithm enabled.")
        print("\n")

        self.index_poly_json = 0

    def to_str_index(self, index):
        if index < 10:
            return "000" + str(index)
        elif index < 100:
            return "00" + str(index)
        elif index < 1000:
            return "0" + str(index)
        else:
            return str(index)

    def process(self, msg):
        """ Extract data from convex_plane_decomposition_msgs.msg import PlanarTerrain

        Args:
            - msg : convex_plane_decomposition_msgs.msg
        """
        surfaces = []
        for region in msg.planarRegions:
            for inset in region.insets:
                # Local to world SE3 transform. Points being expressed in the local frame of the surfaces
                # ie. z = 0 along the normal of the surface.
                t0 = clock()
                wMl = self.fromMessageParameters(region.plane_parameters)
                t1 = clock()
                print("Params [ms] : ", 1000 * (t1 - t0))
                wMl.translation[2] += self.offset_z

                # No holes inside the planar region
                if len(inset.holes) == 0:
                    # Decimate the number of points
                    try:
                        print("--no-holes--")
                        t0 = clock()
                        outer_boundary_simplify = self.simplify(inset.outer_boundary.points, self.threshold)
                        t1 = clock()
                        print("Simplify  [ms] : ", 1000 * (t1 - t0))

                        # import json
                        # filename = "/home/thomas_cbrs/Desktop/log_file_gazebo/debug/expe_02/poly_" + self.to_str_index(self.index_poly_json) + ".json"
                        # polygon_reshaped_json = [[pt.x,pt.y] for pt in outer_boundary_simplify]
                        # with open(filename, 'w') as file:
                        #     json.dump(polygon_reshaped_json, file)
                        # self.index_poly_json += 1
                        # print("size of object : ", len(outer_boundary_simplify))
                        t0 = clock()
                        res = self.algorithm.decomposePoly(outer_boundary_simplify)
                        t1 = clock()
                        print("Decompose [ms] : ", 1000 * (t1 - t0))
                        t0 = clock()
                        for polygon in res:
                            surfaces.append(self.toWorldFrame(polygon, wMl))
                        t1 = clock()
                        print("WorldFram [ms] : ", 1000 * (t1 - t0))
                    except:
                        print("Decomposition failed (no holes), algorithm used : " + self.DECOMPO_ALGO.name)
                        from IPython import embed
                        embed()
                        if self.has_security:
                            print("Trying with " + self.security_type.name + " algorithm...")
                            try:
                                outer_boundary_simplify = self.simplify(inset.outer_boundary.points, self.threshold)
                                res = self.security_algorithm.decomposePoly(outer_boundary_simplify)
                                for polygon in res:
                                    surfaces.append(self.toWorldFrame(polygon, wMl))
                            except:
                                print("Decomposition using security algorithm failed.")

                # Holes inside the planar region
                else:
                    # print("size of polygon : ", len(inset.outer_boundary.points))
                    # for hole in inset.holes :
                    #     print("size of hole : ", len(hole.points))
                    try:
                        print("--holes--")
                        # Decimate the number of points
                        t0 = clock()
                        polygon = self.simplify(inset.outer_boundary.points, self.threshold)
                        t1 = clock()
                        print("Simplify  [ms] : ", 1000 * (t1 - t0))

                        holes = []
                        t0 = clock()
                        for hole in inset.holes:
                            # Get convex hull (necessary for Tess, otherwise erros)
                            if self.DECOMPO_ALGO == DECOMPO_ALGO.Tess2 or self.convexHoles:
                                hole_hull = self.get_convexHUll(hole.points)
                                # Decimate the remaining shape.
                                holes.append(self.simplify(hole_hull, self.threshold))
                            else:
                                holes.append(self.simplify(hole.points, self.threshold))
                        t1 = clock()
                        print("Get-holes [ms] : ", 1000 * (t1 - t0))

                        # import json
                        # polygon_reshaped = [Point(point[0], point[1]) for point in self.earcut.removeHoles(polygon,holes)]
                        # print("size of object : ", len(polygon_reshaped))
                        # filename = "/home/thomas_cbrs/Desktop/log_file_gazebo/debug/expe_02/poly_" + self.to_str_index(self.index_poly_json) + ".json"
                        # polygon_reshaped_json = [[pt.x,pt.y] for pt in polygon_reshaped]
                        # with open(filename, 'w') as file:
                        #     json.dump(polygon_reshaped_json, file)
                        # self.index_poly_json += 1

                        t0 = clock()
                        res = self.algorithm.decomposePoly(polygon, holes)
                        t1 = clock()
                        print("Decompose [ms] : ", 1000 * (t1 - t0))

                        t0 = clock()
                        for polygon in res:
                            surfaces.append(self.toWorldFrame(polygon, wMl))
                        t1 = clock()
                        print("WorldFram [ms] : ", 1000 * (t1 - t0))

                    except:
                        print("Decomposition failed (with holes), algorithm used : " + self.DECOMPO_ALGO.name)
                        from IPython import embed
                        embed()
                        if self.has_security:
                            try:
                                print("Trying with " + self.security_type.name + " algorithm...")
                                # Decimate the number of points
                                polygon = self.simplify(inset.outer_boundary.points, self.threshold)
                                holes = []
                                for hole in inset.holes:
                                    # Get convex hull (necessary for Tess, otherwise erros)
                                    if self.security_type == DECOMPO_ALGO.Tess2 or self.convexHoles:
                                        hole_hull = self.get_convexHUll(hole.points)
                                        # Decimate the remaining shape.
                                        holes.append(self.simplify(hole_hull, self.threshold))
                                    else:
                                        holes.append(self.simplify(hole.points, self.threshold))

                                # polygon_json = [[pt.x,pt.y] for pt in polygon]
                                # holes_json = [[[hx.x,hx.y] for hx in hole] for hole in holes]

                                res = self.security_algorithm.decomposePoly(polygon, holes)
                                for polygon in res:
                                    surfaces.append(self.toWorldFrame(polygon, wMl))
                            except:
                                print("Decomposition using security algorithm failed.")

        return dict(zip([str(k) for k in range(len(surfaces))], [sf for sf in surfaces]))

    def toWorldFrame(self, polygon, wMl):
        """ Express polygon inside world frame. 

        Args:
            - polygon (list) : List of Point() type. (2D, expressed in local plan, z = 0 normal of the plan)
            - wMl (pinocchio SE3) : local to wrold transform SE3. 
        """
        vertices_3D = []
        for point in polygon:
            vertices_3D.append((np.dot(wMl.rotation, np.array([point.x, point.y, 0.])) + wMl.translation).tolist())
        return vertices_3D

    def simplify(self, points, threshold):
        """ Decimate the number of point using visvalingamwyatt algorithm.
        """
        # points = [[point.x, point.y] for point in hole.points ]
        simplifier = vw.Simplifier([[point.x, point.y] for point in points])

        return [Point(point[0], point[1]) for point in simplifier.simplify(threshold=threshold)]

    def get_convexHUll(self, polygon):
        """ Return the convex hull of a polygon.
        """
        hull = ConvexHull([[point.x, point.y] for point in polygon])
        polygon_hull = []
        for j in hull.vertices:
            polygon_hull.append(Point(hull.points[j][0], hull.points[j][1]))
        return polygon_hull

    def fromMessageParameters(self, plane_parameters):
        """ Parameters from elevation map
        """
        translation = np.array([plane_parameters.position.x, plane_parameters.position.y, plane_parameters.position.z])
        quat = pin.Quaternion.Identity()
        quat.x = plane_parameters.orientation.x
        quat.y = plane_parameters.orientation.y
        quat.z = plane_parameters.orientation.z
        quat.w = plane_parameters.orientation.w
        wMl = pin.SE3(quat, translation)

        return wMl

    def get_contour(self, polygon):
        """ Format the polygon for Tess algorithm.
        Args:
            - polygon (list): List of Points().
        Returns :
            - list: The list containing the vertices of the contour line such as
                            [x0,y0,x1,y1, ... , xn,yn].
        """
        contour = []  # Contour representation [x0,y0,x1,y1, ... , xn,yn]
        for point in polygon:
            contour.append(point[0])
            contour.append(point[1])
        return contour

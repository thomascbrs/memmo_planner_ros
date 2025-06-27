#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
import ndcurves
from geometry_msgs.msg import Point, Pose

class WalkgenVisualizer:
    def __init__(self,
                 topic: str = 'walkgen/trajectories',
                 frame_id: str = 'map',
                 line_width: float = 0.01,
                 point_scale: float = 0.02):
        """
        Publisher for visualizing foot trajectories in RViz.

        :param topic: RViz topic to publish MarkerArray on
        :param frame_id: coordinate frame of the trajectories
        :param line_width: width of the trajectory lines
        """
        self._pub = rospy.Publisher(topic, MarkerArray, queue_size=1)
        self._frame_id = frame_id
        self._line_width = line_width
        self._point_scale = point_scale
        self._colors = [
            ColorRGBA(0.0, 1.0, 0.0, 1.0),
            ColorRGBA(0.0, 0.0, 1.0, 1.0),
            ColorRGBA(1.0, 1.0, 0.0, 1.0),
            ColorRGBA(1.0, 0.0, 0.0, 1.0),
        ]
        rospy.sleep(0.1)  # allow publisher to register

    def visualize(self, coeffs_list):
        """
        Publish the given bezier-polynomial coeff trajectories for each foot,
        deleting the previous set first.

        :param coeffs_list: List of contact schedules, each with 4 feet,
                            each foot is e.g. a list of (t0, coeff_matrix) pairs.
        """
        # 1) Send a MarkerArray with a single DELETEALL to clear old markers:
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        delete_all.header.frame_id = self._frame_id
        delete_all.header.stamp = rospy.Time.now()
        self._pub.publish(MarkerArray(markers=[delete_all]))

        markers = MarkerArray()
        marker_id = 0

        # 2) For each schedule in the queue (optional), or just latest:
        #    here we take only the first (current) schedule:
        coeffs_list = [coeffs_list[0]]
        for foot_idx in range(4):
            # gather all sample points along all phases for this foot
            points = []
            for schedule in coeffs_list:
                phases = schedule[foot_idx]
                for (t0, mat) in phases:
                    # mat is degree×3 array of polynomial coeffs for x,y,z
                    # sample the curve densely, e.g. N = 10 points per phase
                    N = 10
                    ts = np.linspace(0., 1.0, N)
                    # evaluate polynomial at each t
                    for t in ts:
                        curve = ndcurves.bezier(mat, 0.0, 1.0)
                        pt = Point()
                        pt.x, pt.y, pt.z = curve(t)
                        points.append(pt)

            # create a LINE_STRIP marker for this foot
            m = Marker()
            m.header.frame_id = self._frame_id
            m.header.stamp = rospy.Time.now()
            m.ns = 'walkgen'
            m.id = marker_id
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            # pose must have a valid quaternion
            m.pose = Pose()
            m.pose.orientation.w = 1.0
            m.scale.x = self._line_width
            m.color = self._colors[foot_idx]
            m.points = points
            markers.markers.append(m)
            marker_id += 1

            # Points marker
            pts_m = Marker()
            pts_m.header.frame_id = self._frame_id
            pts_m.header.stamp = rospy.Time.now()
            pts_m.ns = 'walkgen_points'
            pts_m.id = marker_id
            pts_m.type = Marker.SPHERE_LIST
            pts_m.action = Marker.ADD
            pts_m.pose = Pose()
            pts_m.pose.orientation.w = 1.0
            pts_m.scale.x = self._point_scale
            pts_m.scale.y = self._point_scale
            pts_m.scale.z = self._point_scale
            pts_m.color = self._colors[foot_idx]
            pts_m.points = points
            markers.markers.append(pts_m)
            marker_id += 1

        # 3) Publish all new markers
        self._pub.publish(markers)

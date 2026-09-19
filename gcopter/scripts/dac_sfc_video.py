#!/usr/bin/env python3
"""Replay a recorded DAC-SFC segment; never invokes a planner or optimizer."""

import json
import math
import os

import rospy
import tf
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def add(a, b):
    return [a[i] + b[i] for i in range(3)]


def sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def mul(a, k):
    return [v * k for v in a]


def dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    length = norm(a)
    return mul(a, 1.0 / length) if length > 1.0e-12 else [1, 0, 0]


def point(v):
    return Point(*[float(x) for x in v])


def poly_faces(planes):
    """Small convex H-polytope: intersect triples, then sort each face."""
    vertices = []
    for i in range(len(planes)):
        for j in range(i + 1, len(planes)):
            for k in range(j + 1, len(planes)):
                a, b, c = planes[i], planes[j], planes[k]
                determinant = dot(a[:3], cross(b[:3], c[:3]))
                if abs(determinant) < 1.0e-9:
                    continue
                x = mul(add(add(mul(cross(b[:3], c[:3]), -a[3]),
                                mul(cross(c[:3], a[:3]), -b[3])),
                            mul(cross(a[:3], b[:3]), -c[3])),
                        1.0 / determinant)
                if any(dot(p[:3], x) + p[3] > 1.0e-5 for p in planes):
                    continue
                if all(norm(sub(x, v)) > 1.0e-5 for v in vertices):
                    vertices.append(x)
    faces = []
    for plane in planes:
        found = [v for v in vertices
                 if abs(dot(plane[:3], v) + plane[3]) < 1.0e-4]
        if len(found) < 3:
            continue
        center = mul([sum(v[d] for v in found) for d in range(3)],
                     1.0 / len(found))
        n = unit(plane[:3])
        u = unit(cross(n, [0, 0, 1] if abs(n[2]) < 0.9 else [0, 1, 0]))
        v = cross(n, u)
        found.sort(key=lambda x: math.atan2(dot(sub(x, center), v),
                                            dot(sub(x, center), u)))
        faces.append(found)
    return faces


def box_quaternion(columns):
    # Eigen's orthogonal eigensystem can be left handed. Flipping an axis
    # preserves the box but gives RViz a valid rotation matrix.
    x, y, z = [list(c) for c in columns]
    if dot(x, cross(y, z)) < 0:
        x = mul(x, -1)
    m = [[x[row], y[row], z[row]] for row in range(3)]
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1) * 2
        return [(m[2][1] - m[1][2]) / s,
                (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s, 0.25 * s]
    i = max(range(3), key=lambda index: m[index][index])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(0, 1 + m[i][i] - m[j][j] - m[k][k])) * 2
    q = [0.0] * 4
    q[i] = 0.25 * s
    q[j] = (m[j][i] + m[i][j]) / s
    q[k] = (m[k][i] + m[i][k]) / s
    q[3] = (m[k][j] - m[j][k]) / s
    return q


class Replay:
    def __init__(self, data):
        self.data = data
        self.center = mul(add(data['a'], data['b']), 0.5)
        self.steps = data['steps']
        self.pub = rospy.Publisher('/dac_sfc_video/markers', MarkerArray,
                                   queue_size=1, latch=True)
        self.tf = tf.TransformBroadcaster()
        self.previous = set()
        self.markers = {}
        self.final_published = False
        self.raw_faces = poly_faces(data['selected_raw_polytope'])
        self.retained_faces = [poly_faces(p) for p in data['retained_corridors']]
        self.preview_count = min(3, len(self.steps))
        self.prune_count = min(2, len(data['removed_candidate_ids']))
        self.prune_start = 34 + 3 * self.preview_count
        self.final_start = self.prune_start + 3 * self.prune_count + 2

    def marker(self, ns, number, kind, color, scale=0.1):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = rospy.Time.now()
        m.ns, m.id, m.type, m.action = ns, number, kind, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = color
        self.markers[(ns, number)] = m
        return m

    def strip(self, ns, number, points, color, width=0.08):
        if len(points) < 2:
            return
        m = self.marker(ns, number, Marker.LINE_STRIP, color, width)
        m.points = [point(p) for p in points]

    def ball(self, ns, number, pos, color, size):
        m = self.marker(ns, number, Marker.SPHERE, color, size)
        m.pose.position = point(pos)

    def caption(self, label):
        m = self.marker('stage_label', 0, Marker.TEXT_VIEW_FACING,
                        (0.08, 0.08, 0.12, 0.95), 0.55)
        m.pose.position = point(add(self.center, [0, 0, 3.0]))
        m.text = label

    def mesh(self, ns, number, faces, color):
        if not faces:
            return
        m = self.marker(ns, number, Marker.TRIANGLE_LIST, color)
        for polygon in faces:
            for i in range(1, len(polygon) - 1):
                m.points.extend((point(polygon[0]), point(polygon[i]),
                                 point(polygon[i + 1])))

    def edges(self, ns, number, faces, color, width=0.025):
        if not faces:
            return
        m = self.marker(ns, number, Marker.LINE_LIST, color, width)
        for polygon in faces:
            for i, p in enumerate(polygon):
                m.points.extend((point(p), point(polygon[(i + 1) % len(polygon)])))

    def plane(self, number, coeff, color):
        n = coeff[:3]
        sq = dot(n, n)
        if sq < 1.0e-12:
            return
        # Project the route midpoint to the plane (valid for arbitrary d).
        center = sub(self.center, mul(n, (dot(n, self.center) + coeff[3]) / sq))
        normal = unit(n)
        u = unit(cross(normal, [0, 0, 1] if abs(normal[2]) < 0.9
                       else [0, 1, 0]))
        v = cross(normal, u)
        radius = min(2.2, max(0.7, norm(self.data['half_widths_ascending'])))
        p0 = add(add(center, mul(u, radius)), mul(v, radius))
        p1 = add(sub(center, mul(u, radius)), mul(v, radius))
        p2 = sub(sub(center, mul(u, radius)), mul(v, radius))
        p3 = add(sub(center, mul(v, radius)), mul(u, radius))
        m = self.marker('separating_planes', number, Marker.TRIANGLE_LIST, color)
        m.points = [point(p) for p in (p0, p1, p2, p0, p2, p3)]

    def draw_axes(self, budget=False):
        vectors = self.data['eigenvectors_columns_ascending']
        radii = self.data['extra_radii_ascending']
        palette = {2: (0.08, 0.7, 0.25, 0.95),
                   1: (1.0, 0.6, 0.03, 0.95),
                   0: (0.72, 0.16, 0.7, 0.95)}
        for index in (2, 1, 0):
            axis = vectors[index]
            length = radii[index] if budget else 1.2
            length = max(0.08, length)
            for side in (-1, 1):
                m = self.marker('directions', 2 * index + (side + 1) // 2,
                                Marker.ARROW, palette[index], 0.055)
                m.scale.y, m.scale.z = 0.11, 0.16
                m.points = [point(self.center),
                            point(add(self.center, mul(axis, side * length)))]
            m = self.marker('direction_labels', index, Marker.TEXT_VIEW_FACING,
                            palette[index], 0.42)
            m.pose.position = point(add(self.center,
                                        mul(axis, length + 0.25)))
            m.text = {2: 'Easy', 1: 'Middle', 0: 'Hard'}[index]

    def domain(self, elapsed):
        m = self.marker('construction_box', 0, Marker.CUBE,
                        (0.12, 0.45, 0.9, 0.13))
        m.pose.position = point(self.center)
        q = box_quaternion(self.data['eigenvectors_columns_ascending'])
        m.pose.orientation.x, m.pose.orientation.y = q[0], q[1]
        m.pose.orientation.z, m.pose.orientation.w = q[2], q[3]
        alpha = min(1.0, max(0.0, (elapsed - 23.0) / 3.0))
        ease = alpha * alpha * (3.0 - 2.0 * alpha)
        widths = self.data['half_widths_ascending']
        m.scale.x, m.scale.y, m.scale.z = [2 * max(0.05, w) *
                                            (0.12 + 0.88 * ease)
                                            for w in widths]

    def witnesses(self, highlighted=None):
        local = self.data['local_obstacles']
        if local:
            m = self.marker('local_obstacles', 0, Marker.SPHERE_LIST,
                            (0.24, 0.27, 0.31, 0.55), 0.095)
            stride = max(1, (len(local) + 2499) // 2500)
            m.points = [point(p) for p in local[::stride]]
        if highlighted is not None:
            ids = highlighted['excluded_obstacle_ids']
            if ids:
                m = self.marker('excluded_obstacles', 0, Marker.SPHERE_LIST,
                                (0.98, 0.65, 0.05, 0.9), 0.15)
                m.points = [point(local[i]) for i in ids[:300]
                            if 0 <= i < len(local)]
            self.ball('active_witness', 0, highlighted['witness'],
                      (1.0, 0.1, 0.07, 1), 0.36)
            self.ball('metric_projection', 0, highlighted['projection'],
                      (0.1, 0.7, 0.95, 1), 0.19)
            self.strip('projection_link', 0,
                       [highlighted['witness'], highlighted['projection']],
                       (0.12, 0.75, 0.9, 1), 0.06)

    def render(self, elapsed):
        if self.final_published:
            self.tf.sendTransform(tuple(self.center), (0, 0, 0, 1),
                                  rospy.Time.now(), 'dac_sfc_focus', 'odom')
            return
        self.markers = {}
        d = self.data
        self.strip('route', 0, d['route'], (0.96, 0.18, 0.17, 0.9), 0.095)
        if elapsed < 4:
            label = '1  Collision-free route'
        else:
            self.strip('probe', 0, d['probe'], (0.06, 0.75, 0.29, 0.9), 0.075)
            label = '2  Route-conditioned MINCO probe'
        if elapsed >= 8:
            self.strip('selected_segment', 0, [d['a'], d['b']],
                       (1.0, 0.8, 0.04, 1.0), 0.2)
            label = '3  Selected route segment'
        if 12 <= elapsed < 23:
            self.draw_axes(elapsed >= 18)
            label = ('5  Measured directional budgets' if elapsed >= 18 else
                     '4  CSGN eigenvectors: large eigenvalue = Easy')
        if 23 <= elapsed < self.final_start:
            self.domain(elapsed)
            label = '6  CSGN-aligned construction domain'
        if 30 <= elapsed < self.final_start:
            self.witnesses()
            label = '7  Obstacles within the construction domain'
        if 34 <= elapsed < self.prune_start:
            index = min(self.preview_count - 1, int((elapsed - 34) / 3))
            if index >= 0:
                self.witnesses(self.steps[index])
                for i in range(index + 1):
                    self.plane(i, self.steps[i]['plane'],
                               (0.12, 0.62, 1.0, 0.25))
                label = '8  Active witness, metric projection, separating face'
        elif 34 <= elapsed < self.final_start and not self.steps:
            label = '8  No obstacle witnesses in this domain'
        if self.prune_start <= elapsed < self.final_start:
            removed = d['removed_candidate_ids']
            step = min(self.prune_count, max(0,
                       int((elapsed - self.prune_start) / 3)))
            for i, plane in enumerate(d['candidate_planes']):
                if i in removed[:step]:
                    continue
                color = ((1.0, 0.15, 0.1, 0.42) if
                         step < self.prune_count and i == removed[step]
                         else (0.12, 0.62, 1.0, 0.18))
                self.plane(i, plane, color)
            label = ('9  Reverse-delete redundant faces' if self.prune_count
                     else '9  All generated faces retained')
        if elapsed >= self.final_start:
            self.mesh('raw_polytope', 0, self.raw_faces,
                      (0.05, 0.54, 0.98, 0.12))
            self.edges('raw_polytope_edges', 0, self.raw_faces,
                       (0.05, 0.45, 0.85, 0.85), 0.045)
            for i, faces in enumerate(self.retained_faces):
                self.edges('retained_corridors', i, faces,
                           (0.18, 0.72, 0.85, 0.33), 0.018)
            color = {'proposed': (0.0, 0.5, 1.0, 1.0),
                     'identity': (1.0, 0.55, 0.0, 1.0),
                     'firi': (0.0, 0.8, 0.2, 1.0),
                     'rils': (1.0, 0.1, 0.1, 1.0)}.get(
                         d['trajectory_method'], (0.0, 0.5, 1.0, 1.0))
            self.strip('optimized_trajectory', 0, d['trajectory'], color, 0.16)
            label = '10  Selected raw polytope and retained corridor'
        self.caption(label)
        current = set(self.markers)
        batch = MarkerArray()
        for ns, number in sorted(self.previous - current):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = rospy.Time.now()
            m.ns, m.id, m.action = ns, number, Marker.DELETE
            batch.markers.append(m)
        batch.markers.extend(self.markers.values())
        self.previous = current
        self.pub.publish(batch)
        if elapsed >= self.final_start:
            self.final_published = True
        self.tf.sendTransform(tuple(self.center), (0, 0, 0, 1),
                              rospy.Time.now(), 'dac_sfc_focus', 'odom')


def main():
    rospy.init_node('dac_sfc_video')
    path = rospy.get_param('~trace_file', '/tmp/dac_sfc_video.json')
    if not os.path.isfile(path):
        rospy.logfatal('DAC-SFC trace file is missing: %s', path)
        return
    with open(path, 'r') as stream:
        data = json.load(stream)
    if data.get('schema') != 1 or data.get('frame') != 'odom':
        rospy.logfatal('Unsupported DAC-SFC trace format')
        return
    expected_seed = rospy.get_param('~map_seed', 42)
    if data['map_seed'] != expected_seed:
        rospy.logfatal('Map seed mismatch: trace=%s launch=%s',
                       data['map_seed'], expected_seed)
        return
    rospy.loginfo('DAC-SFC replay: segment=%s, witness rounds=%s, '
                  'redundant faces=%s', data['segment_id'], len(data['steps']),
                  len(data['removed_candidate_ids']))
    replay = Replay(data)
    speed = max(0.05, float(rospy.get_param('~speed', 1.0)))
    delay = max(0.0, float(rospy.get_param('~start_delay_s', 5.0)))
    start = rospy.Time.now().to_sec() + delay
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        replay.render(max(0.0, (rospy.Time.now().to_sec() - start) * speed))
        rate.sleep()


if __name__ == '__main__':
    main()

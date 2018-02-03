#!/usr/bin/env python2

import rospy
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
import numpy.linalg as la

import copy


def find_kappa(p1, p2, p3):
    p1 = np.array([p1.pose.position.x, p1.pose.position.y])
    p2 = np.array([p2.pose.position.x, p2.pose.position.y])
    p3 = np.array([p3.pose.position.x, p3.pose.position.y])

    v1 = (p2 - p1)
    v2 = (p3 - p2)
    inner_prod = np.dot(v1, v2) / (la.norm(v1) * la.norm(v2))
    ang = np.arccos(np.clip(inner_prod, -1, 1))
    curvature = ang / la.norm(v1)
    return curvature


def make_sphere(p, marker_id):
    global frame_id
    global color_sphere
    global size_sphere

    m = Marker()
    m.ns = 'sphere'
    m.id = marker_id
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time.now()
    m.action = Marker.ADD
    m.type = Marker.SPHERE
    m.color.r = color_sphere[0]
    m.color.g = color_sphere[1]
    m.color.b = color_sphere[2]
    m.color.a = color_sphere[3]
    m.pose = p.pose
    m.scale.x = size_sphere
    m.scale.y = size_sphere
    m.scale.z = size_sphere
    return m


def make_label(p, kappa, marker_id):
    global in_radius
    global frame_id
    global color_sphere
    global size_text
    global offset_text

    m = Marker()
    m.ns = 'label'
    m.id = marker_id
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time.now()
    m.action = Marker.ADD
    m.type = Marker.TEXT_VIEW_FACING
    if in_radius:
        m.text = '{0:5.3}'.format(1.0 / kappa)
    else:
        m.text = '{0:6.4}'.format(kappa)
    m.color.r = color_text[0]
    m.color.g = color_text[1]
    m.color.b = color_text[2]
    m.color.a = color_text[3]
    # Move the label a bit
    text_pose = copy.deepcopy(p.pose)
    text_pose.position.x += offset_text[0]
    text_pose.position.y += offset_text[1]
    text_pose.position.z += offset_text[2]
    m.pose = text_pose
    m.scale.z = size_text
    return m


def publish_marker(bad_kappas):
    global marker_pub
    global show_labels

    marker_id = 0
    ma = MarkerArray()
    for p, kappa in bad_kappas:
        ma.markers.append(make_sphere(p, marker_id))
        if show_labels:
            ma.markers.append(make_label(p, kappa, marker_id))
        marker_id += 1
    marker_pub.publish(ma)


def path_cb(msg):
    global kappa_max
    global marker_pub

    clear_ma = MarkerArray()
    clear_marker = Marker()
    clear_marker.header.frame_id = frame_id
    clear_marker.action = Marker.DELETEALL
    clear_ma.markers.append(clear_marker)
    marker_pub.publish(clear_ma)

    if len(msg.poses) == 0:
        return

    p = msg.poses
    bad_kappas = []
    for i in range(1, len(p) - 2):
        kappa = find_kappa(p[i - 1], p[i], p[i + 1])
        if kappa > kappa_max:
            bad_kappas.append((p[i], kappa))

    if len(bad_kappas) == 0:
        rospy.loginfo('All points in the path were OK!')
    else:
        publish_marker(bad_kappas)


rospy.init_node('find_max_curvature_node')

kappa_max = rospy.get_param('~kappa_max', 0.2)
show_labels = rospy.get_param('~enable_text', True)
in_radius = rospy.get_param('~show_radius', True)
frame_id = rospy.get_param('frame_id', 'map')
color_sphere = rospy.get_param('color_sphere', [0.996, 0.426, 0.641, 0.5])
color_text = rospy.get_param('color_text', [1.0, 1.0, 1.0, 1.0])
size_sphere = rospy.get_param('size_sphere', 0.6)
size_text = rospy.get_param('size_text', 0.8)
if in_radius:
    offset_text_default = [1.0, 0, 0.5]
else:
    offset_text_default = [1.5, 0, 0.5]
offset_text = rospy.get_param('offset_text', offset_text_default)

path_sub = rospy.Subscriber('sPath', Path, path_cb, queue_size=1)
marker_pub = rospy.Publisher('bad_kappa', MarkerArray, queue_size=1)

rospy.spin()

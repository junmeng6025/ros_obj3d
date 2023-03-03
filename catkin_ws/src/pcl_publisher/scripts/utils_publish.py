import numpy as np

import rospy
import tf as ros_tf
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

# EGO BBOX
BBOX_EGO = np.array([[2.15, 0.9, -1.73], [2.15, -0.9, -1.73], [-1.95, -0.9, -1.73], [-1.95, 0.9, -1.73],
                     [2.15, 0.9, -0.23], [2.15, -0.9, -0.23], [-1.95, -0.9, -0.23], [-1.95, 0.9, -0.23]])

# corners of the box: front surface 0-1-5-4-0
#     6 -------- 7
#    /|         /|
#   5 -------- 4 .
#   | |        | |
#   . 2 -------- 3
#   |/         |/
#   1 -------- 0
EGO_LINES = [[0, 1], [1, 2], [2, 3], [3, 0]]  # lower surface
EGO_LINES += [[4, 5], [5, 6], [6, 7], [7, 4]]  # upper surface
EGO_LINES += [[4, 0], [5, 1], [6, 2], [7, 3]]  # connect lower and upper
EGO_LINES += [[2, 7], [6, 3]]                  # cross the front surface

#     7 -------- 3
#    /|         /|
#   4 -------- 2 .
#   | |        | |
#   . 5 -------- 6
#   |/         |/
#   1 -------- 0

# OBJ BBOX
LINES = [[0, 1], [1, 5], [5, 6], [6, 0]]  # lower surface
LINES += [[2, 4], [4, 7], [7, 3], [3, 2]]  # upper surface
LINES += [[2, 0], [4, 1], [7, 5], [3, 6]]  # connect lower and upper surface
# LINES += [[4, 0], [2, 1]]


def publish_pcl(pcl_pub, points, frameID):
    msg = PointCloud2()
    msg.header.stamp = rospy.Time().now()
    msg.header.frame_id = frameID

    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        msg.height = 1
        msg.width = len(points)

    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * points.shape[0]
    msg.is_dense = False
    msg.data = np.asarray(points, np.float32).tostring()

    pcl_pub.publish(msg)


def publish_3d_bbox(bbox3d_pub, vert_list, rgb, frameID, lifetime, line_width=0.05):
    marker_array = MarkerArray()

    for j in range(int(len(vert_list)/8)):
        vert = vert_list[j * 8: (j + 1) * 8]
        marker = Marker()
        marker.header.frame_id = frameID
        marker.header.stamp = rospy.Time.now()

        marker.id = j
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(
            lifetime)  # *2 improve the smoothy
        marker.type = Marker.LINE_LIST

        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.color.a = 1.0
        marker.scale.x = line_width  # line width

        marker.points = []
        for l in LINES:
            p1 = vert[l[0]]
            marker.points.append(Point(p1[0], p1[1], p1[2]))
            p2 = vert[l[1]]
            marker.points.append(Point(p2[0], p2[1], p2[2]))

        q = ros_tf.transformations.quaternion_from_euler(
            0, 0, 0)  # default quaternion
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        marker_array.markers.append(marker)

    bbox3d_pub.publish(marker_array)


def publish_ego_bbox(egobbox_pub, rgb, frameID, line_width=0.1, ego_verts=BBOX_EGO):

    marker = Marker()
    marker.header.frame_id = frameID
    marker.header.stamp = rospy.Time.now()

    marker.id = -1
    marker.action = Marker.ADD
    marker.lifetime = rospy.Duration()
    marker.type = Marker.LINE_LIST

    marker.color.r = rgb[0]
    marker.color.g = rgb[1]
    marker.color.b = rgb[2]
    marker.color.a = 1.0
    marker.scale.x = line_width  # line width

    marker.points = []
    for l in EGO_LINES:
        p1 = ego_verts[l[0]]
        marker.points.append(Point(p1[0], p1[1], p1[2]))
        p2 = ego_verts[l[1]]
        marker.points.append(Point(p2[0], p2[1], p2[2]))

    q = ros_tf.transformations.quaternion_from_euler(
        0, 0, 0)  # default quaternion
    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

    egobbox_pub.publish(marker)

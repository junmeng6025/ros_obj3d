#!/usr/bin/env python3
import rospy
import rospkg
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from pathlib import Path
from utils_data_grob import *
from utils_publish import *

PKG_PATH = rospkg.RosPack().get_path('pcl_publisher')
# https://wiki.ros.org/Packages#Client_Library_Support
print("PKG PATH: "+PKG_PATH)

"""
ATTENTION:
the .obj data folder MUST be located in the ros package folder, 
so that they could be read as Marker msg.
"""
SCENE_ID = "cp_vis_scene"
OURS_DIR = PKG_PATH+"/"+SCENE_ID+"/ours/"
BASE_DIR = PKG_PATH+"/"+SCENE_ID+"/baseline/"
FRAME_LS = find_framelist(OURS_DIR)
"""
DATA ORGANIZATION:
each scene is basically divided into "ours" and "baseline",
under each of them, each frame is a folder, containing the .obj files for:
    - point cloud       (xxx_points.obj)
    - predicted result  (xxx_pred.obj)
    - ground truth      (xxx_gt.obj)

what are published from these .obj files:
    /pcl:       /baseline/<frame_id>/xxx_points.obj
    /gt_bbox:   /baseline/<frame_id>/xxx_gt.obj
    /bl_bbox:   /baseline/<frame_id>/xxx_pred.obj
    /ours_bbox: /ours/<frame_id>/xxx_pred.obj

EXAMPLE: 
cp_vis_scene4
├── baseline
│   ├── n008-2018-08-30-15-31-50-0400__LIDAR_TOP__1535657531048384
│   │   ├── n008-2018-08-30-15-31-50-0400__LIDAR_TOP__1535657531048384_gt.obj
│   │   ├── n008-2018-08-30-15-31-50-0400__LIDAR_TOP__1535657531048384_points.obj
│   │   └── n008-2018-08-30-15-31-50-0400__LIDAR_TOP__1535657531048384_pred.obj
│   ├── ...
│   └── ... (the next frames)
└── ours
    ├── n008-2018-08-30-15-31-50-0400__LIDAR_TOP__1535657531048384
    │   ├── n008-2018-08-30-15-31-50-0400__LIDAR_TOP__1535657531048384_gt.obj     >>> useless
    │   ├── n008-2018-08-30-15-31-50-0400__LIDAR_TOP__1535657531048384_points.obj >>> useless
    │   └── n008-2018-08-30-15-31-50-0400__LIDAR_TOP__1535657531048384_pred.obj
    ├── ...
    └── ... (the next frames)
"""


FRAME_ID = "map"
QUEUE_SIZE = 1
FPS = 5
LIFE_TIME = 1.0/FPS


def talker():
    # declare publishers
    pcl_pub = rospy.Publisher('/pcl', PointCloud2, queue_size=QUEUE_SIZE)
    gtbbox_pub = rospy.Publisher(
        '/gt_bbox', MarkerArray, queue_size=QUEUE_SIZE)
    blbbox_pub = rospy.Publisher(
        '/bl_bbox', MarkerArray, queue_size=QUEUE_SIZE)
    oursbbox_pub = rospy.Publisher(
        '/ours_bbox', MarkerArray, queue_size=QUEUE_SIZE)
    egobbox_pub = rospy.Publisher(
        '/ego_bbox', Marker, queue_size=QUEUE_SIZE)

    # reday to publish
    rospy.init_node('pointcloud_publisher_node', anonymous=True)
    rate = rospy.Rate(FPS)

    frame = 0
    rospy.loginfo("starting... package path: %s" % PKG_PATH)

    while not rospy.is_shutdown():
        framedir = FRAME_LS[frame]
        framedir_path = OURS_DIR + framedir
        bl_framedir_path = BASE_DIR + framedir

        # publish .obj as Marker
        # ######### POINT CLOUD
        points = read_pclobj(
            framedir_path+"/", find_data(framedir_path, "points"))
        publish_pcl(pcl_pub, points, FRAME_ID)

        # ######### OUR PRED
        ours_bbox = read_verts(
            framedir_path+"/" + find_data(framedir_path, "pred"))
        publish_3d_bbox(oursbbox_pub, ours_bbox,
                        [1.0, 0.0, 0.0], FRAME_ID, LIFE_TIME)

        # ######### GROUND TRUTH
        gt_bbox = read_verts(
            framedir_path+"/" + find_data(framedir_path, "gt"))
        publish_3d_bbox(gtbbox_pub, gt_bbox,
                        [0.0, 1.0, 0.0], FRAME_ID, LIFE_TIME, 0.1)

        # ######### BASELINE
        bl_bbox = read_verts(
            bl_framedir_path+"/" + find_data(framedir_path, "pred"))
        publish_3d_bbox(blbbox_pub, bl_bbox,
                        [0.0, 0.0, 1.0], FRAME_ID, LIFE_TIME)

        # ######### EGO
        publish_ego_bbox(egobbox_pub, [0.0, 1.0, 1.0], FRAME_ID)

        rospy.loginfo(
            "publishing [%05d of %05d] ... %s" % (frame, len(FRAME_LS), framedir))

        # node spin
        rate.sleep()
        frame += 1
        if frame >= len(FRAME_LS):
            frame = 0


if __name__ == '__main__':
    talker()

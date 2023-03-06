# ros_obj3d
Visualize the point cloud and 3d BBox from .obj file.
## Run
```bash
cd ros_obj3d/catkin_ws
```
```bash
source devel/setup.bash
```
```bash
roslaunch pcl_publisher scene_publisher.py
```

## .obj file organization:
the .obj data folder MUST be located in the ros package folder, so that they could be read as `Marker` msg.
> /catkin_ws/src/pcl_publisher/cp_vis_scene

```bash
cp_vis_scene
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
```

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


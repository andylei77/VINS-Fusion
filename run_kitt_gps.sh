roslaunch vins vins_rviz.launch 

rosrun global_fusion global_fusion_node

rosrun vins kitti_gps_test /home/andy/selfdrivingcar/catkin_ws_2/src/VINS-Fusion/config/kitti_raw/kitti_10_03_config.yaml /media/andy/ce949beb-8a64-4d55-8ab9-31e5c4f6cf67/kitti/kitti_raw/2011_10_03/2011_10_03_drive_0027_sync


roslaunch vins vins_rviz.launch

rosrun loop_fusion loop_fusion_node /home/andy/selfdrivingcar/catkin_ws_2/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml

rosrun vins kitti_odom_test /home/andy/selfdrivingcar/catkin_ws_2/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml '/media/andy/ce949beb-8a64-4d55-8ab9-31e5c4f6cf67/kitti/kitti_vo/gray/00'

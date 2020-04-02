roslaunch vins vins_rviz.launch

rosrun vins vins_node /home/andy/selfdrivingcar/catkin_ws_2/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml

rosrun loop_fusion loop_fusion_node /home/andy/selfdrivingcar/catkin_ws_2/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml

rosbag play MH_01_easy.bag

#!/bin/bash
#while true; do
#pidstat -I | grep $(pidof dragonfly_vpserver_pc) >>pidload.csv
#sleep 0.1
#done

while true; do
    top -b -n1 | grep kitti_gps_ >> cpu_kitti_gps_vio_node.txt
    top -b -n1 | grep global_fus >> cpu_viogps_global_fusion_node.txt

    top -b -n1 | grep kitti_odom >> cpu_kitti_odom_vio_node.txt
    top -b -n1 | grep loop_fusio >> cpu_loop_fusion_node.txt

    top -b -n1 | grep vins_node >> cpu_vio_node.txt
    #top -b -n1 | grep loop_fusio >> cpu_loop_fusion_node.txt

3.402098
done

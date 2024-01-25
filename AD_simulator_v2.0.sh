#!/bin/bash
gnome-terminal -- bash -c "source /home/ave/.bashrc && roscore; exec bash" &
sleep 2
gnome-terminal -- bash -c"source /home/ave/.bashrc && ai28_rviz" &
sleep 2
gnome-terminal -- bash -c"source /home/ave/.bashrc && av_rviz" &
sleep 2
gnome-terminal -- bash -c"source /home/ave/.bashrc && cd /home/ave/ros/catkin_ws_yoon && sds && rosrun lidar_nciss_ros lidar_ciss_node.py" &
sleep 2
gnome-terminal -- bash -c"source /home/ave/.bashrc && sub_and_write" &
sleep 2

for itr in {1..5}; do
    gnome-terminal -- bash -c"source /home/ave/.bashrc && carla_ros_bridge_a" &
    sleep 2
    gnome-terminal -- bash -c"source /home/ave/.bashrc && traffic_a" &
    sleep 2
    gnome-terminal -- bash -c"source /home/ave/.bashrc && av_normal"
    sleep 2
    pkill -f carla_ros_bridge_a
    sleep 2
    pkill -f traffic_a
    sleep 2
    gnome-terminal -- bash -c"source /home/ave/.bashrc && carla_ros_bridge_b" &
    sleep 2
    gnome-terminal -- bash -c"source /home/ave/.bashrc && traffic_b" &
    sleep 2
    gnome-terminal -- bash -c"source /home/ave/.bashrc && av_normal"
    sleep 2
    pkill -f carla_ros_bridge_b
    sleep 2
    pkill -f traffic_b
    sleep 2
    gnome-terminal -- bash -c"source /home/ave/.bashrc && carla_ros_bridge_c" &
    sleep 2
    gnome-terminal -- bash -c"source /home/ave/.bashrc && traffic_c" &
    sleep 2
    gnome-terminal -- bash -c"source /home/ave/.bashrc && av_normal"
    sleep 2
    pkill -f carla_ros_bridge_c
    sleep 2
    pkill -f traffic_c
    sleep 2
    gnome-terminal -- bash -c"source /home/ave/.bashrc && carla_ros_bridge_d" &
    sleep 2
    gnome-terminal -- bash -c"source /home/ave/.bashrc && traffic_d" &
    sleep 2
    gnome-terminal -- bash -c"source /home/ave/.bashrc && av_normal"
    sleep 2    
    pkill -f carla_ros_bridge_d
    sleep 2
    pkill -f traffic_d
    sleep 2
    gnome-terminal -- bash -c"source /home/ave/.bashrc && carla_ros_bridge_a" &
    sleep 2
    gnome-terminal -- bash -c"source /home/ave/.bashrc && traffic_a" &
    sleep 2
    gnome-terminal -- bash -c"source /home/ave/.bashrc && av_abnormal"
    sleep 2
    pkill -f carla_ros_bridge_a
    sleep 2
    pkill -f traffic_a
    sleep 2
    gnome-terminal -- bash -c"source /home/ave/.bashrc && carla_ros_bridge_b" &
    sleep 2
    gnome-terminal -- bash -c"source /home/ave/.bashrc && traffic_b" &
    sleep 2
    gnome-terminal -- bash -c"source /home/ave/.bashrc && av_abnormal"
    sleep 2
    pkill -f carla_ros_bridge_b
    sleep 2
    pkill -f traffic_b
    sleep 2   
    gnome-terminal -- bash -c"source /home/ave/.bashrc && carla_ros_bridge_c" &
    sleep 2
    gnome-terminal -- bash -c"source /home/ave/.bashrc && traffic_c" &
    sleep 2
    gnome-terminal -- bash -c"source /home/ave/.bashrc && av_abnormal"
    sleep 2
    pkill -f carla_ros_bridge_c
    sleep 2
    pkill -f traffic_c
    sleep 2     
    gnome-terminal -- bash -c"source /home/ave/.bashrc && carla_ros_bridge_d" &
    sleep 2
    gnome-terminal -- bash -c"source /home/ave/.bashrc && traffic_d" &
    sleep 2
    gnome-terminal -- bash -c"source /home/ave/.bashrc && av_abnormal"
    sleep 2
    pkill -f carla_ros_bridge_d
    sleep 2
    pkill -f traffic_d
    sleep 2
done

pkill gnome-terminal
sleep 2

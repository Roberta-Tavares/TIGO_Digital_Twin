#!/usr/bin/env bash
# Script para iniciar a ponte de comunicação ROS 2 <-> Ignition Gazebo do TIGO

source /opt/ros/humble/setup.bash

echo "============================================================"
echo "    Iniciando ROS 2 <-> Ignition Gazebo Bridge (TIGO)      "
echo "============================================================"
echo "Tópicos integrados:"
echo "  [ROS 2 -> GZ]  /cmd_vel"
echo "  [GZ -> ROS 2]  /ascamera_hp60c/camera_publisher/depth0/image_raw"
echo "  [GZ -> ROS 2]  /ascamera_hp60c/camera_publisher/depth0/camera_info"
echo "  [GZ -> ROS 2]  /model/tigo/odometry"
echo "============================================================"

ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist \
  /ascamera_hp60c/camera_publisher/depth0/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image \
  /ascamera_hp60c/camera_publisher/depth0/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo \
  /model/tigo/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry

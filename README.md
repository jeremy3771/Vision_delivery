# TEAM_Vision Delivery Robot

## Overview

> This repository contains the software stack for an autonomous delivery robot system. The repo integrates various components for localization, navigation, path following, and motor control, designed to operate efficiently on resource-constrained computing platforms such as NVIDIA Jetson.

## Components

### 1. delivery_motion

> A ROS2 node for controlling the robot's in-wheel motors and Dynamixel steering servos. This module features a velocity smoothing algorithm that gradually increases or decreases speed to ensure smooth acceleration and deceleration by reducing jerk.

### 2. FAST_LIO_ROS2 (Submodule)

> FAST-LIO (Fast LiDAR-Inertial Odometry) is a powerful algorithm that integrates LiDAR and IMU data to achieve precise localization in various environments. FAST_LIO is containerized using Docker.

#### 2.1 build container
##### Dockerfile
```shell
FROM arm64v8/ros:humble #ROS2 in ARM64 arch
```
##### container_run.sh
```shell
docker run --runtime=nvidia \ #nvidia-docker -> docker & --runtime=nvidia
           --privileged -it \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -e NVIDIA_VISIBLE_DEVICES=all \
           --volume="$PROJECT_DIR:/root/ros2_ws/src" \
           --volume=/data/LIDAR_dataset:/root/data \
           --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
           --net=host \
           --ipc=host \
           --pid=host \
           --shm-size=4gb \
           --name="$CONTAINER_NAME" \
           --env="DISPLAY=$DISPLAY" \
           "$IMAGE_NAME" /bin/bash
```

### 3. regulated_pure_pursuit

> An implementation of the Regulated Pure Pursuit algorithm for path following. This controller allows the robot to follow predefined paths while regulating velocity based on:
> - Path curvature
> - Look-ahead distance

> This algorithm provides more stable path tracking than traditional Pure Pursuit by incorporating additional control parameters.

### 4. pointcloud_to_laserscan

> Converts 3D point cloud data from LiDAR sensors into 2D laser scan messages. This conversion is essential for utilizing 2D-based alignment.  
> The module is specifically designed to support the **robot_alignment** component's requirements for accurate initial pose estimation.

### 5. robot_alignment

> Ensures the robot starts autonomous navigation from a consistent initial pose.

### 6. roi_pointcloud

> This module significantly reduces computational load while maintaining sufficient data for navigation tasks.
> - Applying Region of Interest (ROI) filtering to focus computation on relevant areas
> - Voxel grid filtering to reduce point cloud density

### 7. teleop_keyboard

> Provides manual control of the robot through keyboard commands.
> - Testing robot movement capabilities
> - Manually positioning the robot
> - Emergency override during development and testing phases

## Requirements

- ROS2 Humble
- PCL (Point Cloud Library)
- Eigen
- NVIDIA Jetson or equivalent computing platform
- LiDAR sensor (LIVOX MID-360)

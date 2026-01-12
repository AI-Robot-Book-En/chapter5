# Chapter 5: Robot Vision

## Overview

This section contains sample programs and supplementary information from Chapter 5 of "Hands-on Introduction to AI Robots" by Demura, Hagiwara, Masutani, and Tan.

## Directory Structure

- [opencv_ros2](opencv_ros2): Sample Programs for OpenCV and ROS2

- [yolov8_ros2](yolov8_ros2): Sample Programs for YOLOv8 and ROS2

## Supplementary Information

- Please confirm that your Ubuntu environment supports camera input beforehand.

- If the following error occurs when starting the camera:
```
Cannot open '/dev/video0': 13, Permission denied
```
Solution:
```
sudo chmod 666 /dev/video0
```

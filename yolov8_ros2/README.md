# YOLOv8 and ROS2 Sample Program

## Overview

- Deep learning object detection program using YOLOv8 and ROS2
- Created and verified on Ubuntu 22.04 and ROS Humble

## Installation

- [opencv_ros2](../opencv_ros2/README.md) installation

- Installing the YOLOv8 software
```
pip3 install ultralytics
pip3 uninstall -y opencv-python
```
Note: open-python is automatically installed with ultralytics. Therefore, you must remove it to avoid conflicts with open-contrib-python.

## Execution

Section 5.7.2: YOLO Object Detection
- Device 1: Launch the USB camera usb_cam package
```
ros2 run usb_cam usb_cam_node_exe
```
- Device 2: Run the program
```
ros2 run yolov8_ros2 object_detection
```
- If the program runs successfully, a new window will appear, displaying the camera image with a colored frame around the detected object.

Section 5.7.2: Estimating the Position of Detected Objects
- Device 1: Launch the ROS node for the Intel RealSense D415 depth camera.
```
ros2 launch realsense2_camera rs_launch.py ​​align_depth.enable:=true
```
- Device 2: Run the program.
```
ros2 run yolov8_ros2 object_detection_tf
```
- If the program runs successfully, a new window will appear, displaying the bounding box of the target object in the depth image.
- The /tf topic will output the 3D position of the object in the camera coordinate system.

Section 5.7.2: Action Server for Object Detection
- Device 1: Launch the ROS node for the Intel RealSense D415 depth camera.
```
ros2 launch realsense2_camera rs_launch.py ​​align_depth.enable:=true
```
- Device 2: Run the program.
```
ros2 run yolov8_ros2 object_detection_action_server
```
- Terminal 3: Call ROS action communication (target object 'cup')
- Search for the target object
```
ros2 action send_goal /vision/command airobot_interfaces/action/StringCommand "{command: find cup}"
```
- Continuously track the target object
```
ros2 action send_goal /vision/command airobot_interfaces/action/StringCommand "{command: track cup}"
```
- Stop object detection processing
```
ros2 action send_goal /vision/command airobot_interfaces/action/StringCommand "{command: stop}"
```

## Help

- This sample program has only been tested on Ubuntu. Windows developers can install Ubuntu on a virtual machine such as VirtualBox or VMware and run the sample program.

## Author

TAN Jeffrey Too Chuan

## History

- 2024-10-10: Updated to Ubuntu 22.04 and ROS Humble
- 2022-08-27: License and documentation improvements

## License

Copyright (c) 2022-2025, TAN Jeffrey Too Chuan
All rights reserved.
This project is licensed under the Apache License 2.0. See the LICENSE file in the root directory of this project.

## References

- https://github.com/ultralytics/ultralytics

# OpenCV and ROS2 Sample Program

## Overview

- Robot Vision Application Using OpenCV and ROS2
- Created and Tested on Ubuntu 22.04 and ROS Humble

## Installation

- Install OpenCV-related packages
```
pip3 install opencv-contrib-python==4.5.5.64
```

- Install the package that interfaces ROS2 and OpenCV
```
sudo apt install ros-humble-vision-opencv
```

- Install the USB camera node package
```
sudo apt install ros-humble-usb-cam
```

- Install the ROS wrapper for the Intel RealSense RGB-D camera
```
sudo apt install ros-humble-realsense2-camera
```

- Set the ROS workspace to `~/airobot_ws`.
```
cd ~/airobot_ws/src
```

- Obtain the ros2_aruco package from JMU-ROBOTICS-VIVA
```
git clone https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco
```

- Obtain the repository containing this package
```
git clone https://github.com/AI-Robot-Book-Humble/chapter5
```

- Obtain the repository containing the package defining the action interface
```
git clone https://github.com/AI-Robot-Book-Humble/chapter2
```

- Build the package
```
cd ~/airobot_ws/
colcon build
```

- Configure the overlay
```
source install/setup.bash
```

## Execution

Section 5.3.1: Image Processing with OpenCV
- Run the program
```
python3 ~/airobot_ws/src/chapter5/opencv_ros2/opencv_ros2/imgproc_opencv.py
```
- Check the results

Section 5.3.2: OpenCV Image Processing in ROS
- Device 1: Launch the usb_cam package for the USB camera
```
ros2 run usb_cam usb_cam_node_exe
```
- Device 2: Run the program
```
ros2 run opencv_ros2 imgproc_opencv_ros
```
- Device 3: Check the results
- Run RQt
```
rqt
```
- Select Plugins → Visualization → Image View twice in the RQt menu to add two Image View plugins.
- Select /image_raw and /result from the plugin topic selection menu, respectively.
- Device 4: Examine the connections between nodes and topics.
- Run rqt_graph
```
rqt_graph
```

Section 5.3.3: Subscribing to Depth Data
- Device 1: Launch the ROS node for the Intel RealSense D415 depth camera
```
ros2 launch realsense2_camera rs_launch.py ​​align_depth.enable:=true
```
- Device 2: Check the topic
```
ros2 topic list
```
- Device 3: Display the color and depth images
- Run RQt
```
rqt
```
- Display the color and depth image topics with the Image View plugin

Section 5.5.1: Canny Edge Detection
- Device 1: Launch the usb_cam package for the USB camera
```
ros2 run usb_cam usb_cam_node_exe
```
- Device 2: Run the program
```
ros2 run opencv_ros2 canny_edge_detection
```
- Device 3: Check the results
- Run RQt
```
rqt
```
- Display the resulting topic/edges_result in the Image View plugin

Section 5.5.2: Face detection using a Haar feature-based cascade classifier
- Device 1: Launch the usb_cam package for the USB camera
```
ros2 run usb_cam usb_cam_node_exe
```
- Device 2: Run the program
```
ros2 run opencv_ros2 face_detection
```
- Device 3: Check the results
- Run RQt
```
rqt
```
- Display the resulting topic/face_detection_result in the Image View plugin

Section 5.6.1: QR code detection
- Device 1: Launch the usb_cam package for the USB camera
```
ros2 run usb_cam usb_cam_node_exe
```
- Device 2: Run the program
```
ros2 run opencv_ros2 qrcode_detector
```
- Device 3: Check the resulting character topic
```
ros2 topic echo /qrcode_detector_data
```
- Device 4: Check the resulting image topic
- Run RQt
```
rqt
```
- Display the resulting image topic /qrcode_detector_result using the Image View plugin

Section 5.6.2: Position and Orientation Estimation Using ArUco Markers
- Device 1: Generate a sample marker image
```
ros2 run ros2_aruco aruco_generate_marker
```
- Device 2: Launch the USB camera usb_cam package
```
ros2 run usb_cam usb_cam_node_exe
```
- Device 3: Run the program
```
ros2 run opencv_ros2 aruco_node_tf
```
- Device 4: Check the resulting /tf topic
```
ros2 topic echo /tf
```
- Device 5: Visualize the marker and camera coordinate system
- Run RViz
```
rviz2
```
- In the RViz window, change Fixed Frame to default_cam and click Add to add the TF.

## Help

- This sample program has only been tested on Ubuntu. Windows developers can install Ubuntu on a virtual machine such as VirtualBox or VMware and run the sample program.

- We have confirmed that an error occurs if both the opencv-python and opencv-contrib-python Python packages are installed.
To avoid this, run the following command:
```
pip3 uninstall opencv-python
```

- When running the aruco_node_tf sample program, coordinate axes are not displayed in the image. Also, when observing the contents of the tf topic, translation remains at 0,0,0 and rotation remains at 0,0,0,1.

Solution: A USB camera calibration result file is required. tf is calculated based on the camera_info from the calibration results.
```
∼/.ros/camera_info/default_cam.yaml
```

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

- https://opencv.org/
- https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco

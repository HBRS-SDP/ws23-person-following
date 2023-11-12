# Person Following Codebase
## Software Development Project
 
Base code for ROS2 based person following robot involving RGB-D camera, laser scanner

Milestone Presentations [Link](https://drive.google.com/drive/folders/1fSUbau2GBS1j4a7OnIZlPYnxty2QNiYR?usp=drive_link) <br>

[Deliverables for API ](https://github.com/HBRS-SDP/ws23-person-following/API_Plan.md) 

### Technologies
- Robot Platform: Robile3
- OS: Ubuntu 22.04 LTS
- Middleware: ROS2 Humble
- Hardware: Intel Realsense D455, Hokuyo URG laser scanner
- Libraries: OpenCV, numpy, mediapipe

### Setup Installation 
#### Prerequisites
- opencv
- mediapipe
- pyrealsense2
- Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

1. Clone this repo in your ```workspace_name/src``` directory:
```
https://github.com/HBRS-SDP/ws23-person-following/
```
2. Install the dependencies
```
sudo apt install python3-dev python3-pip python3-numpy build-essential libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev gfortran openexr libatlas-base-dev
sudo apt install libopencv-dev python3-opencv
pip3 install opencv-python
pip3 install mediapipe
pip install pyrealsense2
```
3. Build the workspace using ```colcon build```

## Launch Gazebo
To launch pose estimation in gazebo using Realsense camera:
```
ros2 launch pose_estimator realsense_pose_estimator.launch.py 
```
To launch pose estimation in gazebo using system webcam:
```
ros2 launch pose_estimator estimate_pose.launch.py
```

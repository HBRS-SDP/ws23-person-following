# Person Following Codebase
## Software Development Project
 
Base code for ROS2 based person following robot involving RGB-D camera, laser scanner <br>
- [Milestone Presentations](https://drive.google.com/drive/folders/1fSUbau2GBS1j4a7OnIZlPYnxty2QNiYR?usp=drive_link) <br>
- [API Architecture](https://github.com/HBRS-SDP/ws23-person-following/blob/main/API_architecture.md)
- [User Test Cases](https://github.com/HBRS-SDP/ws23-person-following/blob/main/User_Test_Cases.md)

### Technologies
- Robot Platform: Robile3
- OS: Ubuntu 22.04 LTS
- Middleware: ROS2 Humble
- Hardware: Intel Realsense D455, Hokuyo URG laser scanner
- Libraries: OpenCV, numpy, mediapipe

### Setup Installation 
Prerequisites
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

```
3. Build the workspace using ```colcon build```.


## To set up the workspace in the robot follow the instructions in the below link:

(Workspace setup in robot [here](https://robile-amr.readthedocs.io/en/humble/source/getting_started.html#cloning-repositories-from-hbrs-amr-group))

Before running any launch files. Download the file ```robot_realsense.launch.py ``` from this repo and put it inside the launch folder in the robot's ROS workspace for example:

```ros2_ws/src/robile/launch```

## To publish to the respective topics from the robot.

Run the below commands in the robot to start the topics:

First set the ros distro to ros humble then,

```
cd ros_ws/ 
source install/local_setup.bash
ros2 launch robile_bringup robot_realsense.launch.py 

```

## Launch file to check the functionality of the realsense camera.
To launch pose estimation using Realsense camera:
```
ros2 launch pose_estimator realsense_pose_estimator.launch.py 
```
## Launch Follow behaviour
To launch the behaviour to follow person:  
```
ros2 launch pose_estimator follow_person.launch.py
```

**NOTE:** The follow behaviour does not work, if the realsense camera is not connected

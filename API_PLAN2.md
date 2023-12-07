### Software Delivery:
The software package is delivered as a Python script structured as a ROS package. It includes dependencies such as the `mediapipe`, `pyrealsense2` for pose estimation, and `rclpy` for ROS integration. The script utilizes the Robot Operating System (ROS) and provides laser-based obstacle avoidance using the `/scan` topic.

### MVP Test Scenario: Following the robot at a threshold distance
#### Triggering the Scenario:
- **Operator Action:**
  - The operator executes the `main()` function of the Python script on the robot.
  - Optionally, the operator places a person within the field of view of the RealSense camera.

#### System Behavior:

- **Initialization:**
  - The system initiates the RealSense pipeline.
  - It starts listening to color and depth streams:
    - `/camera/camera/color/image_raw`: Provides color image data from the RealSense camera.
    - `/camera/camera/depth/image_rect_raw`: Supplies depth image data from the RealSense camera.
  - Subscribes to the `/scan` topic for laser data.
  - Subscribes to the `/joy` topic for joystick input.

- **Sensor Fusion:**
  - The system fuses data from pose estimation and laser scans to make informed decisions about robot movement.

- **Obstacle Avoidance:**
  - If obstacles are detected in the laser scan data, the robot adjusts its movement to avoid collisions (`/scan`).
  - The obstacle avoidance behavior is integrated with the existing pose-based following behavior.

- **Robot Movement:**
  - Based on the combined information from pose estimation and laser scans, the robot publishes `Twist` messages to the '/cmd_vel' topic.

- **Visualization:**
  - The system displays the RealSense color frames with pose landmarks overlaid in the 'Mediapipe Feed' window.
  - Visualizations related to laser scans and obstacle avoidance can be added to help the operator monitor the system.

- **Termination:**
  - The operator can trigger the termination of the script by pressing 'q' in the 'Mediapipe Feed' window.
  - Upon termination, the RealSense pipeline stops, and the visualization window is closed.

---

#### Data Received from Subscribed Topics:

- `/camera/camera/color/image_raw`: Provides color images in a raw format from the RealSense camera. These images are processed for pose estimation and visualization.
- `/camera/camera/depth/image_rect_raw`: Offers depth images rectified from the RealSense camera. These depth images are utilized to calculate distances for obstacle avoidance and following behaviors.
- `/scan`: Provides laser scan data used for obstacle detection and avoidance.
- `/joy`: Supplies joystick input used to control certain functionalities or behaviors of the system.

---

## Pose Estimation and Navigation API Documentation

### `PoseEstimationNode`

A ROS node responsible for pose estimation and navigation using RealSense camera data and joystick input.

#### Attributes:
- `publisher_`: Publisher object for sending robot movement commands (`/cmd_vel`).
- `scan_subscription`: Subscription object for receiving laser scan data (`/scan`).
- `joy_subscription`: Subscription object for receiving joystick input (`/joy`).
- `safe_min_range`: Float - Defines the safe minimum range for collision avoidance.

#### Methods:
- `joy_callback(joy_msg)`: Processes joystick input.
- `scan_callback(scan_msg)`: Handles laser scan data for collision avoidance.
- `camera_callback(image_msg)`: Processes camera data for pose estimation.
- `depth_callback(depth_msg)`: Processes depth camera data for depth calculation.
- `process_pose(image)`: Performs pose estimation using `mediapipe`.
- `publish_movement_commands()`: Publishes movement commands based on pose estimation and laser scans.
- `publish_stop_command()`: Publishes stop command for the robot.
- `render_detections(image, landmarks)`: Renders pose landmarks on the camera feed for visualization.

### Custom Data Structures:

#### ROS Messages:
- `Twist` message: Represents linear and angular velocities for robot control.
- Laser scan data: Received in the standard `LaserScan` message format.
- Camera data: Received in the standard `Image` message format.

#### Data Types:
- Pose landmarks: Utilizes data types from the `mediapipe` library.
- Depth information: Processed using OpenCV and `CvBridge`.
- Laser scan data processing: Utilizes NumPy for array manipulation.

### Additional Classes and Functions:

- `main()`: Initializes the ROS node and executes pose estimation and navigation functionality.

### Additional Considerations:

- **Error Handling:**
  - Exception handling for missing frames, pose estimation failures, or issues with laser scan data should be implemented. Errors should be logged or printed for debugging purposes.
  
- **Documentation:**
  - In-code comments are crucial for understanding and maintaining the codebase.
  - Update the user manual to include information about laser-based obstacle avoidance and any additional parameters or configurations.

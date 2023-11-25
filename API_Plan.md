
### Software Delivery:
The software package is delivered as a Python script structured as a ROS package. It includes dependencies such as the `mediapipe`, `pyrealsense2` for pose estimation, and `rospy` for ROS integration. The script utilizes the Robot Operating System (ROS) and provides laser-based obstacle avoidance using the `/scan` topic.

### MVP Test Scenario: Following the robot at a threshold distance
#### Triggering the Scenario:
- **Operator Action:**
  - The operator executes the `main()` function of the Python script on the robot.
  - Optionally, the operator places a person within the field of view of the RealSense camera.

#### System Behavior:

- **Initialization:**
  - The system initiates the RealSense pipeline.
  - It starts listening to color and depth streams.
  - Subscribes to the `/scan` topic for laser data.

- **Sensor Fusion:**
  - The system fuses data from pose estimation and laser scans to make informed decisions about robot movement.

- **Obstacle Avoidance:**
  - If obstacles are detected in the laser scan data, the robot adjusts its movement to avoid collisions.
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

### API Descriptions:

#### Inputs/Outputs:

- **ROS Topics:**
  - **Inputs:**
    - Color and depth streams from the RealSense camera.
    - Laser scan data from the `/scan` topic.
  - **Outputs:**
    - `Twist` messages published to the '/cmd_vel' topic for robot control.

#### Custom Data Structures:

- **ROS Messages:**
  - The `Twist` message is used to convey linear and angular velocities for robot control.
  - Laser scan data is received in the standard `LaserScan` message format.

- **Data Types:**
  - Pose landmarks and depth information are represented using appropriate data types from the `mediapipe` and `pyrealsense2` libraries.
  - Laser scan data is processed using standard Python and ROS data types.

- **Classes:**
  - The script defines a class `PoseEstimationNode` that encapsulates the functionality of the pose estimation node.
  - It also utilizes classes from the `mediapipe` and `pyrealsense2` libraries.
  - Additional classes or functions can be added for laser-based obstacle avoidance.

---

### Pose Estimation and Navigation API Documentation

#### `PoseEstimationNode`

A ROS node handling pose estimation and navigation.

- **Attributes:**
  - `publisher_`: Publisher object for robot movement commands
  - `scan_subscription`: Subscription object for laser scan data
  - `joy_subscription`: Subscription object for joystick input
  - `safe_min_range`: Float - Safe minimum range for collision avoidance
  
- **Methods:**
  - `joy_callback(joy_msg)`: Processes joystick input
  - `scan_callback(scan_msg)`: Uses laser scan data for collision avoidance
  - `run(ranges)`: Executes the pose estimation and navigation loop

---

#### `follow_motion(depth_at_person, person_x)`

A class handling following a detected person.

- **Parameters:**
  - `depth_at_person`: Float - Depth at detected person
  - `person_x`: Float - X-coordinate of the detected person
  
- **Methods:**
  - `setup(**kwargs)`: Initializes the behavior
  - `update()`: Updates the motion based on the distance to the person

---

#### `move_to_wall(slope_angle)`

A class for moving the robot towards a wall.

- **Parameters:**
  - `slope_angle`: Float - Angle of the wall relative to the robot
  
- **Methods:**
  - `setup(**kwargs)`: Initializes the behavior
  - `update()`: Moves the robot towards the wall

---

#### `align_to_wall(slope_angle, align)`

A class for aligning the robot parallel to the wall.

- **Parameters:**
  - `slope_angle`: Float - Angle of the wall relative to the robot
  - `align`: Float - Alignment angle
  
- **Methods:**
  - `setup(**kwargs)`: Initializes the behavior
  - `update()`: Adjusts the robot's orientation to align with the wall

---

#### `follow_wall(last_pos, rotation_direction, process_laser, point_min_dist, dist, align)`

A class for following along a wall.

- **Parameters:**
  - `last_pos`: Float - Last known position of the detected person
  - `rotation_direction`: Float - Direction of rotation
  - `process_laser`: Numpy array - Laser scan data
  - `point_min_dist`: Float - Minimum distance from the point
  - `dist`: Float - Distance to the point
  - `align`: Float - Alignment angle
  
- **Methods:**
  - `setup(**kwargs)`: Initializes the behavior
  - `update()`: Controls robot movement while following the wall

---

#### `run_online(process_laser, sigma, k)`

This function converts polar to Cartesian coordinates and performs online line detection.

- **Parameters:**
  - `process_laser`: Numpy array - Laser scan data
  - `sigma`: Float - Standard deviation for filtering
  - `k`: Integer - Parameter for line detection
  
- **Returns:** 
  - If lines found, returns distance (`d`) and slope (`m`) of the nearest line; otherwise, returns 0.

---

#### `main()`

Main function to initialize the ROS node and run the pose estimation and navigation functionality.

- **Parameters:** None
- **Returns:** None

#### Custom Data Structures:

- **ROS Messages:**
  - The `Twist` message is used to convey linear and angular velocities for robot control.
  - Laser scan data is received in the standard `LaserScan` message format.

- **Data Types:**
  - Pose landmarks and depth information are represented using appropriate data types from the `mediapipe` and `pyrealsense2` libraries.
  - Laser scan data is processed using standard Python and ROS data types.

- **Classes:**
  - The script defines a class `PoseEstimationNode` that encapsulates the functionality of the pose estimation node.
  - It also utilizes classes from the `mediapipe` and `pyrealsense2` libraries.
  - Additional classes or functions can be added for laser-based obstacle avoidance.

### Additional Considerations:

- **Error Handling:**
  - The script should handle errors gracefully, such as absence of valid frames, pose estimation failures, or issues with laser scan data.
  - Errors should be logged or printed for debugging purposes.

- **Documentation:**
  - In-code documentation should be provided using comments.
  - Update the user manual to include information about laser-based obstacle avoidance and any additional parameters or configurations.




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

## Pose Estimation and Navigation API Documentation

### `PoseEstimationNode`

A ROS node responsible for pose estimation and navigation using RealSense camera data and joystick input.

#### Attributes:
- `publisher_`: Publisher object for sending robot movement commands.
- `scan_subscription`: Subscription object for receiving laser scan data.
- `joy_subscription`: Subscription object for receiving joystick input.
- `safe_min_range`: Float - Defines the safe minimum range for collision avoidance.

#### Methods:
- `joy_callback(joy_msg)`: Processes joystick input.
- `scan_callback(scan_msg)`: Handles laser scan data for collision avoidance.
- `run(ranges)`: Executes the pose estimation and navigation loop using RealSense camera data and laser scans.

### Custom Data Structures:

#### ROS Messages:
- `Twist` message: Represents linear and angular velocities for robot control.
- Laser scan data: Received in the standard `LaserScan` message format.

#### Data Types:
- Pose landmarks and depth information: Utilizes appropriate data types from the `mediapipe` and `pyrealsense2` libraries.
- Laser scan data processing: Relies on standard Python and ROS data types.

### Additional Classes and Functions:

- `follow_motion(depth_at_person, person_x)`: Handles following a detected person.
- `move_to_wall(slope_angle)`: Facilitates moving the robot towards a wall.
- `align_to_wall(slope_angle, align)`: Manages aligning the robot parallel to the wall.
- `follow_wall(last_pos, rotation_direction, process_laser, point_min_dist, dist, align)`: Handles the process of following along a wall.
- `run_online(process_laser, sigma, k)`: Converts polar to Cartesian coordinates and performs an algorithm to find the nearest wall.
- `main()`: Initializes the ROS node and executes pose estimation and navigation functionality.

### Additional Considerations:

- **Error Handling:**
  - The script should handle exceptions gracefully, particularly concerning missing frames, pose estimation failures, or issues with laser scan data. Errors should be logged or printed for debugging purposes.
  
- **Documentation:**
  - In-code documentation using comments is essential for understanding and maintaining the codebase.
  - Ensure the user manual is updated to include information about laser-based obstacle avoidance and any additional parameters or configurations.

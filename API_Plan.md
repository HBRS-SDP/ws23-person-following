# Software Delivery
The software package is delivered as a Python script structured as a ROS package. It includes dependencies such as `mediapipe`, `pyrealsense2` for pose estimation, and `rclpy` for ROS integration. The script utilizes the Robot Operating System (ROS) and provides laser-based obstacle avoidance using the `/scan` topic.

## MVP Test Scenario: 
- Robot is following the human at a threshold distance
### Triggering the Scenario:
- **Operator Action:**
  - The operator executes the `main()` function of the Python script on the robot.
  - Optionally, the operator places a person within the field of view of the RealSense camera.

### System Behavior:
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
  - Based on the combined information from pose estimation and laser scans, the robot publishes `Twist` messages to the `/cmd_vel` topic.

- **Visualization:**
  - The system displays the RealSense color frames with pose landmarks overlaid in the 'Mediapipe Feed' window.
  - Visualizations related to laser scans and obstacle avoidance can be added to help the operator monitor the system.

- **Termination:**
  - The operator can trigger the termination of the script by pressing 'q' in the 'Mediapipe Feed' window.
  - Upon termination, the RealSense pipeline stops, and the visualization window is closed.

### API Descriptions:
#### Inputs/Outputs:
- **ROS Topics:**
  - **Inputs:**
    - Color and depth streams from the RealSense camera.
    - Laser scan data from the `/scan` topic.
  - **Outputs:**
    - `Twist` messages published to the `/cmd_vel` topic for robot control.

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
  - The script should handle errors gracefully, such as the absence of valid frames, pose estimation failures, or issues with laser scan data.
  - Errors should be logged or printed for debugging purposes.

- **ROS2 Quality Tools:**
  - Using ROS2 quality tools, including `roslint` and `ament_lint`, to ensure high code quality.
  - Integrating these tools in the development workflow to enforce coding standards and best practices.

- **Static Analysis:**
  - Run static analysis tools regularly to catch potential issues early in the development process.
  - Static analysis helps identify coding errors, stylistic issues, and potential sources of bugs before runtime.
- **Documentation:**
  - In-code documentation should be provided using comments.
  - Update the user manual to include information about laser-based obstacle avoidance and any additional parameters or configurations.


### Test Case 1:

**Given:**
- The robot is powered on.
- The ROS environment is set up correctly.
- A person is within the field of view of the RealSense camera.

**When:**
- The `main()` function is executed, running the `PoseEstimationNode`.

**Then:**
- The robot should initiate the RealSense pipeline.
- Pose estimation should be performed on the detected person.
- The robot should publish `Twist` messages to `/cmd_vel` based on the person's position and depth information.
- Calculate and print the expected turn.
- The robot should move forward if the person is farther than the desired distance, stop if at the desired distance, and move backward if closer than the desired distance.

### Test Case 2:

**Given:**
- The robot is powered on.
- The ROS environment is set up correctly.
- No person is within the field of view of the RealSense camera.

**When:**
- The `main()` function is executed, running the `PoseEstimationNode`.

**Then:**
- The robot should initiate the RealSense pipeline.
- Since no person is detected, print "No person detected!".
- The robot should not publish any `Twist` messages.

### Test Case 3:

**Given:**
- The robot is powered on.
- The ROS environment is set up correctly.
- The RealSense camera does not provide valid color or depth frames.

**When:**
- The `main()` function is executed, running the `PoseEstimationNode`.

**Then:**
- The robot should handle the absence of valid frames gracefully.
- Pose estimation should not be performed.
- The robot should not publish any `Twist` messages.

### Test Case 4:

**Given:**
- The robot is powered on.
- The ROS environment is set up correctly.
- The operator triggers the termination of the script by pressing 'q' in the 'Mediapipe Feed' window.

**When:**
- The `main()` function is executed, running the `PoseEstimationNode`.

**Then:**
- The robot should stop the RealSense pipeline and close the 'Mediapipe Feed' window when the 'q' key is pressed.

### Test Case 5:

**Given:**
- The robot is powered on.
- The ROS environment is set up correctly.
- The person is within the field of view of the RealSense camera.
- Depth information is not available.

**When:**
- The `main()` function is executed, running the `PoseEstimationNode`.

**Then:**
- The robot should handle the absence of depth information gracefully.
- Print a message indicating the absence of depth information.
- The robot should not publish any `Twist` messages.

### Test Case 6:

**Given:**
- The robot is powered on.
- The ROS environment is set up correctly.
- The person is within the field of view of the RealSense camera.
- Pose estimation fails.

**When:**
- The `main()` function is executed, running the `PoseEstimationNode`.

**Then:**
- The robot should handle pose estimation failures gracefully.
- Print a message indicating the failure.
- The robot should not publish any `Twist` messages.

### Test Case 7:

**Given:**
- The robot is powered on.
- The ROS environment is set up correctly.
- Multiple persons are within the field of view of the RealSense camera.

**When:**
- The `main()` function is executed, running the `PoseEstimationNode`.

**Then:**
- The robot should handle multiple persons within the frame.
- Select and follow the closest person.
- Print appropriate messages regarding the selected person's position and movement.

### Test Case 8:

**Given:**
- The robot is powered on.
- The ROS environment is set up correctly.
- The person is within the field of view of the RealSense camera.
- The person moves dynamically.

**When:**
- The `main()` function is executed, running the `PoseEstimationNode`.

**Then:**
- The robot should continuously update the person's position based on real-time pose estimation.
- Adjust its movement accordingly and continuously print the updated expected turn.

### Test Case 9:

**Given:**
- The robot is powered on.
- The ROS environment is set up correctly.
- The person is initially outside the field of view and enters later.

**When:**
- The `main()` function is executed, running the `PoseEstimationNode`.

**Then:**
- The robot should handle the dynamic entry of a person into the frame.
- Start tracking the person's movements once detected and print relevant information.

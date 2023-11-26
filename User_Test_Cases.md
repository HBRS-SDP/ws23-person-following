### Test Case 1:

**Given:**
- The robot is powered ON.
- The ROS environment is set up correctly.
- No person is within the field of view of the RealSense camera.

**When:**
- The `main()` function is executed, running the `PoseEstimationNode`.

**Then:**
- The robot should initiate the RealSense pipeline.
- Since no person is detected, print "No person detected!".
- The robot should not publish any Twist messages.

### Test Case 2:

**Given:**
- The robot is powered ON.
- The ROS environment is set up correctly.
- A person is within the field of view of the RealSense camera.

**When:**
- The `main()` function is executed, running the `PoseEstimationNode`.

**Then:**
- The robot should initiate the RealSense pipeline.
- Pose estimation should be performed on the detected person.
- The robot should publish Twist messages to `/cmd_vel` based on the person's position and depth information.
- The robot should move forward if the person is farther than the desired distance, stop if at the desired distance

### Test Case 3:

**Given:**
- The robot is powered ON.
- The ROS environment is set up correctly.
- The person is within the field of view of the RealSense camera.
- Realsense Camera does not publish depth frames or valid RGB frames.

**When:**
- The `main()` function is executed, running the `PoseEstimationNode`.

**Then:**
- The robot should interpret the absence of depth information.
- Print a message indicating the absence of depth information.
- The robot should not publish any Twist messages.

### Test Case 4:

**Given:**
- The robot is powered ON.
- The ROS environment is set up correctly.
- The person is within the field of view of the RealSense camera.
- Pose estimation fails.

**When:**
- The `main()` function is executed, running the `PoseEstimationNode`.

**Then:**
- The robot should show interpret that pose estimation has failed.
- Print a message indicating the failure.
- The robot should not publish any Twist messages.

### Test Case 5:

**Given:**
- The robot is powered ON.
- The ROS environment is set up correctly.
- Multiple persons are within the field of view of the RealSense camera.

**When:**
- The `main()` function is executed, running the `PoseEstimationNode`.

**Then:**
- The robot should handle multiple persons within the frame.
- Select and track the closest person.
- Start following the person selected.
- Print appropriate messages regarding the selected person's position and movement.

### Test Case 6:

**Given:**
- The robot is powered ON.
- The ROS environment is set up correctly.
- The person is within the field of view of the RealSense camera.
- The person moves dynamically.

**When:**
- The `main()` function is executed, running the `PoseEstimationNode`.

**Then:**
- The robot should continuously update the person's position based on real-time pose estimation.
- Adjust its movement accordingly and continuously print the updated expected turn.

### Test Case 7:

**Given:**
- The robot is powered ON.
- The ROS environment is set up correctly.
- The person is initially outside the field of view and enters later.

**When:**
- The `main()` function is executed, running the `PoseEstimationNode`.

**Then:**
- The robot should handle the dynamic entry of a person into the frame.
- Start tracking the person's movements once detected and print relevant information.


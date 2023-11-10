import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp
import numpy as np
import pyrealsense2 as rs
from geometry_msgs.msg import Twist

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose

        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Start the pipeline
        self.pipeline.start(config)

        # Create a publisher to control the robot's movement
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize variables for occlusion behavior
        self.last_known_pose = None
        self.occlusion_detected = False

    def run(self):
        with self.mp_pose.Pose(min_detection_confidence=0.4, min_tracking_confidence=0.4) as pose:
            while True:
                # Wait for a frame from RealSense camera
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue

                # Convert the color frame to a NumPy array
                color_image = np.asanyarray(color_frame.get_data())

                # Recolor image to RGB
                image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                image.flags.writeable = False

                # Make detection
                results = pose.process(image)

                # Recolor back to BGR
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                # Extract landmarks
                try:
                    landmarks = results.pose_landmarks.landmark

                    left_hip = landmarks[mp_pose.PoseLandmark.LEFT_HIP]
                    right_hip = landmarks[mp_pose.PoseLandmark.RIGHT_HIP]

                    if landmarks:  # Check if landmarks are available

                        # Calculating the center of the detected person
                        self.person_x = (left_hip.x + right_hip.x) / 2

                        print("The center of the found person: ", self.person_x)

                        width, height = image.shape[1], image.shape[0]
                        center_x, center_y = width // 2, height // 2
                        self.depth_at_person = depth_frame.get_distance(center_x, center_y)

                        # Check for occlusion
                        if self.depth_at_person == 0.0:
                            self.occlusion_detected = True
                            print("Occlusion detected!")

                        msg = Twist()
                        desired_distance = 1.0
                        linear_vel = 0.4
                        angular_vel = 0.65

                        # Adjust behavior based on occlusion detection
                        if not self.occlusion_detected:
                            if self.depth_at_person > desired_distance:
                                # Move forward
                                msg.linear.x = linear_vel
                                msg.angular.z = (0.5 - self.person_x) * angular_vel
                            elif self.depth_at_person < desired_distance:
                                # Move backward
                                msg.linear.x = -linear_vel
                            else:
                                # Stop
                                msg.linear.x = 0.0
                                msg.angular.z = 0.0
                        else:
                            # Perform occlusion behavior
                            self.adjust_orientation_and_movement(msg)

                        self.publisher_.publish(msg)

                    else:
                        print("No person detected!")

                except Exception as e:
                    print(f"Exception: {e}")

                # Render detections
                self.mp_drawing.draw_landmarks(image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS,
                                               self.mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
                                               self.mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2))

                cv2.imshow('Mediapipe Feed', image)

                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break

        self.pipeline.stop()
        cv2.destroyAllWindows()

    def adjust_orientation_and_movement(self, msg):
        # Implement logic for adjusting orientation and movement during occlusion
        # Set the desired orientation and movement based on the last known pose
        if self.last_known_pose:
            # Calculate desired angular velocity and linear velocity
            desired_angular_vel = 0.1  # Adjust as needed
            desired_linear_vel = 0.2  # Adjust as needed

            # Calculate the difference between the current pose and the last known pose
            # Adjust the robot's joints (e.g., head or camera) to rotate in the direction of the stored pose

            # Set the command velocity
            msg.linear.x = desired_linear_vel
            msg.angular.z = desired_angular_vel

        else:
            # No last known pose available, stop the robot
            msg.linear.x = 0.0
            msg.angular.z = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()
    node.run()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

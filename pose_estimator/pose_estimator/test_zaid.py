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

        # Initialize memory for last known person's position
        self.last_known_position = None

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
                    # Calculating the center of the detected person

                    self.person_x = (left_hip.x + right_hip.x) / 2
                    width, height = image.shape[1], image.shape[0]
                    center_x, center_y = width // 2, height // 2
                    self.depth_at_person = depth_frame.get_distance(center_x, center_y)

                    self.last_known_position = self.person_x  # Update last known position
                    self.last_distance = self.depth_at_person

                    if landmarks and self.depth_at_person:  # Check if landmarks and distances are available

                        msg = Twist()
                        desired_distance = 1.0
                        linear_vel = 0.4
                        angular_vel = 0.65
                        # expected_turn = (0.5 - self.person_x) * angular_vel

                        if self.depth_at_person > desired_distance:
                            # Move forward
                            msg.linear.x = linear_vel
                            msg.angular.z = (0.5 - self.person_x) * angular_vel
                        
                        if self.depth_at_person == desired_distance:
                            # Move forward
                            msg.linear.x = 0.0
                            msg.angular.z = 0.0

                        elif self.depth_at_person < desired_distance:
                            # Move backward
                            msg.linear.x = -linear_vel

                        self.publisher_.publish(msg)

                    else:
                        if self.last_known_position is not None:
                            msg = Twist()
                            desired_distance = 0.0
                            linear_vel = 0.4
                            angular_vel = 0.65
                            # Move based on the last known position
                            while self.depth_at_person > desired_distance:
                                # Move forward
                                # msg.linear.x = linear_vel
                                print("Moving forward")

                            if self.last_known_position < 0.4:
                                # msg.linear.x = 0.0  # Stop moving forward
                                # msg.angular.z = angular_vel  # Turn right
                                print("Moving right to check on the person")
                            elif self.last_known_position > 0.6:
                                # msg.linear.x = 0.0  # Stop moving forward
                                # msg.angular.z = -angular_vel  # Turn left
                                print("Moving left to check on the person")

                        else:
                            # No person detected or out of sight
                            msg.linear.x = 0.0  # Stop moving
                            msg.angular.z = 0.0

                        self.publisher_.publish(msg)

                except Exception as e:
                    pass

                # Render detections
                self.mp_drawing.draw_landmarks(image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS,
                                               self.mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
                                               self.mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2))

                cv2.imshow('Mediapipe Feed', image)

                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break

        self.pipeline.stop()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()
    node.run()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

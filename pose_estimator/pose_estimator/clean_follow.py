import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp
from sensor_msgs.msg import Joy
import numpy as np
import pyrealsense2 as rs
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy

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
        self.pipeline.start(config)

        # Create a publisher to control the robot's movement
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.safe_min_range = 0.25
        self.person_x = None
        self.depth_at_person = None
        self.point_at_min_dist = None
        # Create a subscriber to receive joystick input
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
    def joy_callback(self, joy_msg):
        # Process joystick input here
        # Example: Extract joystick axes and buttons
        self.buttons = joy_msg.buttons


    def scan_callback(self, msg):
        # Use laser scan data for collision avoidance
        self.ranges = msg.ranges
        if self.ranges:
            laser = np.array(self.ranges)
            laser = np.nan_to_num(laser, nan=0.0)
            laser[laser <= 0.02] = 1.0
            self.point_at_min_dist = min(laser)

    def run(self):
        with self.mp_pose.Pose(min_detection_confidence=0.4, min_tracking_confidence=0.4) as pose:
            while rclpy.ok():
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

                    left_hip = landmarks[mp.solutions.pose.PoseLandmark.LEFT_HIP]
                    right_hip = landmarks[mp.solutions.pose.PoseLandmark.RIGHT_HIP]

                    if landmarks:
                        self.person_x = (left_hip.x + right_hip.x) / 2
                        width, height = image.shape[1], image.shape[0]
                        center_x, center_y = width // 2, height // 2
                        self.depth_at_person = depth_frame.get_distance(center_x, center_y)

                        msg = Twist()
                        desired_distance = 1.0
                        linear_vel = 0.4
                        angular_vel = 0.65

                        # if self.point_at_min_dist < self.safe_min_range:
                        #     msg.linear.x = 0.0
                        #     msg.angular.z = 0.0
                        #     self.publisher_.publish(msg)
                        #     print("Avoiding obstacle")

                        if self.depth_at_person > desired_distance:
                            # Move forward
                            msg.linear.x = linear_vel
                            msg.angular.z = (0.5 - self.person_x) * angular_vel

                        elif abs(self.depth_at_person - desired_distance) <= 0.1:
                            # Stop at desired distance
                            msg.linear.x = 0.0
                            msg.angular.z = 0.0

                        elif self.depth_at_person < desired_distance:
                            # Move backward
                            msg.linear.x = 0.0
                            msg.angular.z = 0.0
                        elif self.buttons[0] != 1:
                            msg.linear.x = 0.0
                            msg.angular.z = 0.0         
                        self.publisher_.publish(msg)

                    else:
                        print("No person detected!")
                        # Stop the robot if no person detected
                        msg = Twist()
                        msg.linear.x = 0.0
                        msg.angular.z = 0.0
                        # Check if the joy button is pressed
                        if self.buttons[0] == 1:  # Check for button press
                            # Stop the robot
                            msg = Twist()
                            msg.linear.x = 0.0
                            msg.angular.z = 0.0
                            self.publisher_.publish(msg)
                        elif self.depth_at_person < desired_distance:
                            # Move backward
                            msg.linear.x = 0.0
                            msg.angular.z = 0.0
                        self.publisher_.publish(msg)
                except Exception as e:
                    print("Exception:", e)

                # Render detections
                self.mp_drawing.draw_landmarks(
                    image, results.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS,
                    self.mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
                    self.mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2)
                )

                cv2.imshow('Mediapipe Feed', image)

                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break

            self.pipeline.stop()
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    pose_estimation_node = PoseEstimationNode()
    pose_estimation_node.run()
    rclpy.spin(pose_estimation_node)
    pose_estimation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

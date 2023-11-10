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

                    if landmarks:  # Check if landmarks are available
                        # Calculate the center of the detected person
                        self.person_x = (landmarks[mp_pose.PoseLandmark.LEFT_HIP].x + landmarks[mp_pose.PoseLandmark.RIGHT_HIP].x) / 2

                        # Update the last known pose
                        self.last_known_pose = {
                            'person_x': self.person_x,
                            'depth_at_person': depth_frame.get_distance(int(self.person_x * image.shape[1]), int(image.shape[0] / 2))
                        }

                        # Reset occlusion flag
                        self.occlusion_detected = False

                    elif self.last_known_pose and not self.occlusion_detected:
                        # Person not detected, but last known pose exists
                        # Implement occlusion behavior
                        self.handle_occlusion()

                except Exception as e:
                    self.get_logger().error(f"Error: {e}")

                # Render detections
                self.mp_drawing.draw_landmarks(image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS,
                                               self.mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
                                               self.mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2))

                cv2.imshow('Mediapipe Feed', image)

                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break

        self.pipeline.stop()
        cv2.destroyAllWindows()

    def handle_occlusion(self):
        # Implement your occlusion behavior here
        # Example: Rotate towards the last known pose and move forward
        msg = Twist()
        msg.linear.x = 0.2  # Set your desired linear velocity
        msg.angular.z = (0.5 - self.last_known_pose['person_x']) * 0.65  # Adjust angular velocity based on the deviation from the center
        self.publisher_.publish(msg)
        self.occlusion_detected = True

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()
    node.run()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

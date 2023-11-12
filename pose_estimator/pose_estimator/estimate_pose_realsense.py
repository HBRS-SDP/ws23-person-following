import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp
import numpy as np
import pyrealsense2 as rs
from geometry_msgs.msg import Twist

class PoseEstimationNode(Node):
    def __init__(self):
        """
        Initializes the PoseEstimationNode class.

        This class performs real-time pose estimation using Mediapipe and processes depth information
        from a RealSense camera for robot control.
        """
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

    def run(self):
        """
        Main loop for pose estimation and robot control.
        """
        with self.mp_pose.Pose(min_detection_confidence=0.3, min_tracking_confidence=0.3) as pose:
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
                    landmarks = results.pose_world_landmarks.landmark

                    # The center pixel's depth value
                    width, height = color_frame.get_width(), color_frame.get_height()
                    center_x, center_y = width // 2, height // 2  # Center of the image
                    # The distance value at the center pixel
                    #depth_value = depth_frame.get_distance(center_x, center_y)

                    if landmarks:
                        print('Landmarks found')
                        depth_value = depth_frame.get_distance(center_x, center_y)
                        print(depth_value)
 

                except Exception as e:
                    # Catch any exceptions that might occur during processing
                    self.get_logger().error(f"Error in pose estimation: {e}")

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
    """
    Main function to initialize and run the PoseEstimationNode.
    """
    rclpy.init(args=args)
    node = PoseEstimationNode()
    node.run()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

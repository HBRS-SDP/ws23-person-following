import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

# Initialize mediapipe drawing utilities and pose modules
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

class PoseEstimationNode(Node):
    def __init__(self):
        """
        Initializes the PoseEstimationNode class.

        This class performs real-time pose estimation using Mediapipe and processes frames
        from the camera to extract landmarks.
        """
        super().__init__('pose_estimation_node')
        self.bridge = CvBridge()  # Initialize the CvBridge for converting ROS images
        self.cap = cv2.VideoCapture(0)  # Initialize a video capture object (camera)

    def run(self):
        """
        Main loop for pose estimation and landmark extraction.
        """
        with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
            while rclpy.ok():  # Continue loop while the ROS node is running
                ret, frame = self.cap.read()  # Read a frame from the camera

                # Recolor image to RGB
                image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image.flags.writeable = False

                # Make pose detection
                results = pose.process(image)

                # Recolor back to BGR
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                # Extract pose landmarks
                try:
                    landmarks = results.pose_world_landmarks.landmark
                    # Print the left elbow landmark position
                    print(landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value])

                except Exception as e:
                    # Handle any exceptions that might occur during processing
                    self.get_logger().error(f"Error in pose estimation: {e}")

                # Render pose landmarks on the image
                mp_drawing.draw_landmarks(
                    image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                    mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
                    mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2)
                )

                cv2.imshow('Mediapipe Feed', image)

                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break

    def destroy_node(self):
        """
        Cleanup function to release the camera and close the OpenCV window.
        """
        self.cap.release()  # Release the video capture object
        cv2.destroyAllWindows()  # Close any OpenCV windows
        super().destroy_node()  # Call the superclass destroy_node method to clean up ROS-related resources

def main(args=None):
    """
    Main function to initialize and run the PoseEstimationNode.
    """
    rclpy.init(args=args)
    node = PoseEstimationNode()
    node.run()
    rclpy.spin(node)
    node.destroy_node()  # Cleanup resources when the ROS node is shutting down
    rclpy.shutdown()

if __name__ == '__main__':
    main()

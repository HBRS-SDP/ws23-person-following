import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.publisher = self.create_publisher(Image, 'pose_estimation_image', 10)

    def run(self):
        while rclpy.ok():
            self.process_frame()

    def process_frame(self):
        ret, frame = self.cap.read()

        if not ret:
            return

        # Recolor image to RGB
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False

        # Make detection
        results = self.pose.process(image)

        # Extract landmarks
        try:
            landmarks = results.pose_landmarks.landmark
            left_elbow = landmarks[mp_pose.PoseLandmark.LEFT_ELBOW]

            x = left_elbow.x
            y = left_elbow.y
            z = left_elbow.z
            print(f"Left Elbow - X: {left_elbow.x}, Y: {left_elbow.y}, Z: {left_elbow.z}")

            # Create an Image message with the processed frame
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(img_msg)

        except:
            pass

        # Render detections
        mp_drawing.draw_landmarks(
            frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
            mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
            mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2)
        )

        cv2.imshow('Mediapipe Feed', frame)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            self.destroy_node()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()
    node.run()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

class MovementControl(Node):
    def __init__(self, goal_distance, distance_threshold):
        super().__init__('movement_control_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_distance = goal_distance
        self.distance_threshold = distance_threshold

    def move(self, z_value):
        twist = Twist()

        # Calculate the desired distance from the goal
        desired_distance = self.goal_distance - z

        # If the robot is too far from the goal, move forward
        if desired_distance > self.distance_threshold:
            twist.linear.x = 0.2  # Adjust the linear velocity as needed
        else:
            twist.linear.x = 0.0  # Stop moving

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    movement_control = MovementControl(goal_distance=0.5, distance_threshold=0.05)  # Adjust goal_distance and threshold as needed
    rclpy.spin(movement_control)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

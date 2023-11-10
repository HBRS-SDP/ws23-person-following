import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp
import numpy as np
import pyrealsense2 as rs
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class FollowPersonNode(Node):
    def __init__(self):
        super().__init__('follow_person_node')
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose

        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Create a publisher for cmd_vel messages
        self.publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber for the camera image
        self.subscriber_image = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()

        # Initialize variables to keep track of the person's position
        self.person_x = 0
        self.depth_at_person = 0
        self.last_known_person_x = 0  # Initialize to the center
        self.person_in_sight = False  # Flag to indicate if the person is in sight

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')
            return

        with self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
            # Wait for a frame from the RealSense camera
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()

            # Convert the color image to RGB
            image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Make detection
            results = pose.process(image)

            try:
                landmarks = results.pose_landmarks
                if landmarks:
                    # Extract the position of the person's left and right hips
                    left_hip = landmarks.landmark[mp.solutions.pose.PoseLandmark.LEFT_HIP]
                    right_hip = landmarks.landmark[mp.solutions.pose.PoseLandmark.RIGHT_HIP]

                    # Calculate the person's center position
                    self.person_x = (left_hip.x + right_hip.x) / 2

                    # Calculate the depth value at the center position
                    width, height = cv_image.shape[1], cv_image.shape[0]
                    center_x, center_y = width // 2, height // 2
                    self.depth_at_person = depth_frame.get_distance(center_x, center_y)

                    # Store the last known person's position when in sight
                    self.last_known_person_x = self.person_x
                    self.person_in_sight = True

                    msg = Twist()
                    desired_distance = 1.0  # Desired distance between the robot and the person

                    # Adjust linear velocity based on the distance error
                    linear_vel = 0.2  # Constant linear velocity (adjust as needed)
                    distance_error = self.depth_at_person - desired_distance
                    msg.linear.x = linear_vel + linear_vel * distance_error

                    # Adjust angular velocity based on the person's position
                    angular_vel = 0.5  # Constant angular velocity (adjust as needed)
                    if self.person_x < 0.4:
                        msg.angular.z = angular_vel  # Turn right if person is on the left
                    elif self.person_x > 0.6:
                        msg.angular.z = -angular_vel  # Turn left if person is on the right
                    else:
                        msg.angular.z = 0.0  # Keep straight if person is in the center

                    self.publisher_cmd_vel.publish(msg)

                else:

                    if self.last_known_person_x:
                        # If the person is not detected, use the last known position
                        if self.person_in_sight:
                            msg = Twist()
                            desired_distance = 1.0  # Desired distance between the robot and the person

                            # Adjust linear velocity based on the distance error
                            linear_vel = 0.2  # Constant linear velocity (adjust as needed)
                            distance_error = self.depth_at_person - desired_distance
                            msg.linear.x = linear_vel + linear_vel * distance_error

                            # Adjust angular velocity based on the last known person's position
                            angular_vel = 0.5  # Constant angular velocity (adjust as needed)
                            if distance_error==0.0:
                                if self.last_known_person_x < 0.4:
                                    msg.angular.z = -angular_vel  # Turn left if person was on the left
                                elif self.last_known_person_x > 0.6:
                                    msg.angular.z = angular_vel  # Turn right if person was on the right
                                else:
                                    msg.angular.z = 0.0  # Keep straight if person was in the center

                                self.publisher_cmd_vel.publish(msg)
                            else:
                                msg.linear.x= linear_vel + linear_vel * distance_error
                                self.publisher_cmd_vel.publish(msg)
                    
                    else:
                        
                        #To stay idle.
                        msg= Twist()
                        msg.linear.x= 0.0
                        msg.linear.y= 0.0
                        msg.angular.z= 0.0
                        self.publisher_cmd_vel.publish(msg)

            except Exception as e:
                self.get_logger().error(f'Error processing pose: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = FollowPersonNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

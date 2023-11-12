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

    def image_callback(self, msg):

        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        # Convert the color image to RGB
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Make detection
        results = pose.process(image)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')
            return

            # Convert the color image to RGB
        with self.mp_pose.Pose(min_detection_confidence=0.4, min_tracking_confidence=0.4) as pose:
            while True:

                image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

                # Make detection
                results = pose.process(image)

                try:
                    landmarks = results.pose_landmarks
                    if landmarks:
                        left_hip = landmarks.landmark[mp.solutions.pose.PoseLandmark.LEFT_KNEE]
                        right_hip = landmarks.landmark[mp.solutions.pose.PoseLandmark.RIGHT_KNEE]

                        self.person_x = (left_hip.x + right_hip.x) / 2

                        print("The center of the person: " , self.person_x)

                        # width, height = cv_image.shape[1], cv_image.shape[0]
                        # center_x, center_y = width // 2, height // 2
                        # self.depth_at_person = depth_frame.get_distance(center_x, center_y)

                
                        # msg = Twist()
                        # desired_distance = 1.0 

                        # linear_vel = 0.2 
                        # distance_error = self.depth_at_person - desired_distance

                        # angular_vel = 0.5 
                        # if self.depth_at_person >1.0 and self.person_x == 0.5:
                        #     msg.linear.x = linear_vel * distance_error
                        #     msg.angular.z = 0.0 # Straight
                        # if self.depth_at_person >1.0 and self.person_x < 0.4:
                        #     msg.angular.z = angular_vel  # Turn right
                        # elif self.depth_at_person >1.0 and self.person_x > 0.6:
                        #     msg.angular.z = -angular_vel  # Turn left


                    # self.publisher_cmd_vel.publish(msg)

                except:
                    pass

                # Render detections
                self.mp_drawing.draw_landmarks(image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS,
                                               self.mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
                                               self.mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2))

                cv2.imshow('Mediapipe Feed', image)

                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break
def main(args=None):
    rclpy.init(args=args)
    node = FollowPersonNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

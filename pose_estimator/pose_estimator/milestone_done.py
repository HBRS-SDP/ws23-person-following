import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Joy, LaserScan, Image
import numpy as np
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose
        self.bridge = CvBridge()

        # Initializing RealSense pipeline
        self.depth_image = None

        # Sublisher to control the robot's movement
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to receive laser scan data
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # Subscriber to receive joystick input
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Subscriber to receive camera data
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.camera_callback,
            10
        )

        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        self.safe_min_range = 0.4
        self.person_x = None
        self.depth_at_person = None
        self.point_at_min_dist = 0.
        self.ranges = None
        self.buttons = []
        self.left_dist = 999999.9 # Left
        self.right_dist = 999999.9 # Right

    def joy_callback(self, joy_msg):
        # Processing joystick input here
        self.buttons = joy_msg.buttons

    def depth_callback(self, depth_msg):
        try:
            # Processing depth camera data for depth calculation
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            self.depth_image = cv_depth_image.copy()
        except Exception as e:
            self.get_logger().info("Exception in depth_callback: {0}".format(e))

    def scan_callback(self, scan_msg):
        # #Uncomment to check the length of the laser array
        # self.ranges = scan_msg.ranges
        # print("Length of laser array: ", len(self.ranges))

        #The values between left and right
        self.ranges = scan_msg.ranges
        laser = np.array(self.ranges)
        laser = np.nan_to_num(laser, nan=0.0)
        laser[laser <= 0.02] = 1.0
        self.point_at_min_dist= min(laser)
                
        self.left_dist = np.mean(np.array(scan_msg.ranges[509:512]))

        self.right_dist = np.mean(np.array(scan_msg.ranges[0:3]))

        # print("LEN",len(self.ranges)) - 513
        print("left_dist--",self.left_dist)
        print("right_dist--",self.right_dist)

        print("Min point: ", self.point_at_min_dist)


    def camera_callback(self, image_msg):
        try:
            # Process camera data for pose estimation
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            self.process_pose(cv_image)
        except Exception as e:
            self.get_logger().info("Exception in camera_callback: {0}".format(e))


    def process_pose(self, image):
        with self.mp_pose.Pose(min_detection_confidence=0.4, min_tracking_confidence=0.4) as pose:
            image.flags.writeable = False
            results = pose.process(image)
            image.flags.writeable = True

            if results.pose_landmarks:
                landmarks = results.pose_landmarks.landmark

                left_hip = landmarks[mp.solutions.pose.PoseLandmark.LEFT_HIP]
                right_hip = landmarks[mp.solutions.pose.PoseLandmark.RIGHT_HIP]

                self.person_x = (left_hip.x + right_hip.x) / 2
                width, height = image.shape[1], image.shape[0]
                center_x, center_y = width // 2, height // 2
                
                if self.depth_image is not None:
                    self.depth_at_person = self.depth_image[center_y, center_x] 

                    if self.depth_at_person > 2.5:

                        print("Person detected too far!")
                        self.publish_stop_command()
                    
                    else:
                        pass
 
                self.publish_movement_commands()

            else:
                # Stop the robot if no person detected
                self.publish_stop_command()

            self.render_detections(image, results.pose_landmarks)

    def publish_movement_commands(self):
        msg = Twist()
        desired_distance = 1.0
        linear_vel = 0.3
        angular_vel = 0.5

        print(type(self.depth_at_person), self.depth_at_person)


        if self.point_at_min_dist < self.safe_min_range:
            self.publish_stop_command()
            print("Avoiding collision", self.point_at_min_dist)

            if self.left_dist< 0.35 and self.right_dist> 0.35:
                msg.linear.x = 0.
                msg.linear.y = 0.15
                msg.angular.z = 0.0
                print("Moving Left")
            
            elif self.left_dist> 0.35 and self.right_dist< 0.35:
                msg.linear.x = 0.
                msg.linear.y = 0.
                msg.angular.z = 0.0
                print("Moving Right")
                
        elif self.depth_at_person > desired_distance:
            # Move forward with left and right motion
            print("Following person")
            msg.linear.x = linear_vel
            msg.angular.z = (0.5 - self.person_x) * angular_vel

            # if self.point_at_min_dist < self.safe_min_range and self.left_dist< 0.35 and self.right_dist> 0.35:
            #     msg.linear.x = 0.
            #     msg.linear.y = 0.15
            #     msg.angular.z = 0.0
            #     print("Moving Left")
            
            # elif self.point_at_min_dist < self.safe_min_range and self.left_dist> 0.35 and self.right_dist< 0.35:
            #     msg.linear.x = 0.
            #     msg.linear.y = 0.
            #     msg.angular.z = 0.0
            #     print("Moving Right")


        elif abs(self.depth_at_person - desired_distance) <= 0.1:
            # Maintaining desired distance
            self.publish_stop_command()

        elif self.depth_at_person < desired_distance:
            # Maintaining desired distance
            self.publish_stop_command()

        elif self.left_dist< 0.35 and self.right_dist< 0.35:

            pass

        self.publisher_.publish(msg)

    def publish_stop_command(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def render_detections(self, image, landmarks):
        self.mp_drawing.draw_landmarks(
            image, landmarks, mp.solutions.pose.POSE_CONNECTIONS,
            self.mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
            self.mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2)
        )
        cv2.imshow('Mediapipe Feed', image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    pose_estimation_node = PoseEstimationNode()
    rclpy.spin(pose_estimation_node)
    pose_estimation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 

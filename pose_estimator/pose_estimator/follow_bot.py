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
        self.publisher_ = self.create_publisher(Twist, '/robot/cmd_vel', 10)

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

                    left_hip = landmarks[mp_pose.PoseLandmark.LEFT_KNEE]
                    right_hip = landmarks[mp_pose.PoseLandmark.RIGHT_KNEE]

                    if landmarks:  # Check if landmarks are available

                        print("Landmarks found")


                        print("Left Hip:", left_hip)

                        self.person_x = (left_hip.x + right_hip.x) / 2

                        print("The center of the person: " , self.person_x)
                        # Get the z-coordinate (distance) of a specific landmark (e.g., LEFT_ELBOW)
                        target_landmark = landmarks[self.mp_pose.PoseLandmark.LEFT_ELBOW.value]
                        target_z = target_landmark.z  # Assuming this is the z-coordinate
# 
                        # Define the desired distance from the target
                        desired_distance = 1.0  # Adjust this value as needed
# 
                        msg = Twist()
                        # Adjust the linear velocity based on the distance error
                        if target_z > desired_distance:
                            print("working")
                            #msg.linear.x = 0.05  # Move forward
                        elif target_z < desired_distance:
                            #msg.linear.x = -0.05  # Move backward
                            self.publisher_.publish(msg)
# 
                        print("Moving towards target distance: ", desired_distance)
# 
# 
                        width, height = image.shape[1], image.shape[0]
                        center_x, center_y = width // 2, height // 2
                        self.depth_at_person = depth_frame.get_distance(center_x, center_y)
# 
                # 
                        msg = Twist()
                        desired_distance = 1.0 
# 
                        linear_vel = 0.2 
                        distance_error = self.depth_at_person - desired_distance
# 
                        angular_vel = 0.5 
                        if self.depth_at_person >1.0 and self.person_x == 0.5:
                            # msg.linear.x = linear_vel * distance_error
                            # msg.angular.z = 0.0 # Straight
                            print('dds')
                        elif self.depth_at_person >1.0 and self.person_x < 0.4:
                            # msg.angular.z = angular_vel  # Turn right
                            print('ddl')
                        elif self.depth_at_person >1.0 and self.person_x > 0.6:
                            # msg.angular.z = -angular_vel  # Turn left
                            print('ddr')

# 
                    self.publisher_cmd_vel.publish(msg)

                except:
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

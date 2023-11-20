import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp
import numpy as np
import pyrealsense2 as rs
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from rclpy.node import Node

point_min_dist=None
laser= None
depth_at_person= None
person_x= None
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
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.run,
            10
        )
        # Create publisher to override joystick commands
        self.joy_pub = self.node.create_publisher(
            msg_type=Joy,
            topic=self.joy_topic,
        )

        self.safe_min_range= 0.25



    def run(self, msg):
        # Use laser scan data for collision avoidance
        self.ranges = msg.ranges
        if self.ranges:
            laser=np.array(self.blackboard.laser_scan)
            laser=np.nan_to_num(laser,nan=0.0)
            laser[laser<=0.02]=1.0

        self.point_at_min_dist= min(laser)

        if self.point_at_min_dist < self.safe_min_range:
            stop_motion(self.point_at_min_dist,laser)

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

                    if landmarks:  # Check if landmarks are available

                        #calculating the centre of the detected person
                        self.person_x = (left_hip.x + right_hip.x) / 2

                        print("The center of the found person: ", self.person_x)

                        width, height = image.shape[1], image.shape[0]
                        center_x, center_y = width // 2, height // 2
                        self.depth_at_person = depth_frame.get_distance(center_x, center_y)
                        #Triggering follow motion
                        follow_motion(self.depth_at_person,self.person_x)

                    else:
                        msg= Twist()
                        msg.linear.x= 0.0
                        msg.linear.y= 0.0
                        msg.angular.z= 0.0
                        self.publisher_.publish(msg)
                        print("No person detected!")
                
                    '''
                    sending self.joy_pub in this method. The frame_id for Joy() message is
                    "/dev/input/js0". It is similar to just pressing the deadman button on the joystick.
                    Nothing to implement here.
                    '''
                    ## Uncomment the following lines to publish Joy() message when running on the ROBILE ##
                    joyMessage = Joy()
                    joyMessage.header.frame_id = "/dev/input/js0"
                    joyMessage.axes = [0., 0., 0., 0., 0., 0.]
                    joyMessage.buttons = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
                    self.joy_pub.publish(joyMessage)        
                    
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


class follow_motion(depth_at_person, person_x):
    def __init__(self, 
                 name: str="stop platform", 
                 topic_name1: str="/cmd_vel", 
                 topic_name2: str="/joy"):
        super(stop_motion, self).__init__(name)
        # Set up topic name publish rotation commands
        self.cmd_vel_topic = topic_name1
        self.joy_topic = topic_name2
        self.depth_at_person = depth_at_person
        self.person_x= person_x


    def setup(self, **kwargs):
        self.logger.info("[STOP MOTION] setting up stop motion behavior")

        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish rotation commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.cmd_vel_topic,
        )

        # Create publisher to override joystick commands
        self.joy_pub = self.node.create_publisher(
            msg_type=Joy,
            topic=self.joy_topic,
        )
        
        self.feedback_message = "setup"
        return True
    
    def update(self):

        msg = Twist()
        desired_distance = 1.0
        linear_vel = 0.4
        angular_vel = 0.65
        expected_turn= (0.5 - self.person_x) * angular_vel

        print("Expected turn",expected_turn)

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



class stop_motion(point_min_dist, laser):

    def __init__(self, 
                 name: str="stop platform", 
                 topic_name1: str="/cmd_vel", 
                 topic_name2: str="/joy"):
        super(stop_motion, self).__init__(name)
        # Set up topic name publish rotation commands
        self.cmd_vel_topic = topic_name1
        self.joy_topic = topic_name2
        self.point_min_dist= point_min_dist
        self.laser= laser

    def setup(self, **kwargs):
        self.logger.info("[STOP MOTION] setting up stop motion behavior")

        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish rotation commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.cmd_vel_topic,
        )

        # Create publisher to override joystick commands
        self.joy_pub = self.node.create_publisher(
            msg_type=Joy,
            topic=self.joy_topic,
        )
        
        self.feedback_message = "setup"
        return True
    
    def update(self):
        '''
        Primary function of the behavior is implemented in this method

        Rotating the ROBILE at maximum allowed angular velocity in a given direction, 
        where if _direction_ is +1, it implies clockwise rotation, and if it is -1, it implies
        counter-clockwise rotation
        '''

        process_laser = np.array(self.laser)

        print("Closest distance",self.point_min_dist)
        self.left_dist = np.mean((np.array(process_laser[509:512])))
        self.front_dist = np.mean((np.array(process_laser[253:256])))
        self.right_dist = np.mean((np.array(process_laser[0:3])))

        print("This is front dist:", self.front_dist)
        print("This is left dist:", self.left_dist)
        print("This is right dist:", self.right_dist)

        '''
        Send the zero rotation command to self.cmd_vel_topic
        '''
        if (self.right_dist > self.left_dist) and self.front_dist<1.2:
            twist_msg = Twist()

            twist_msg.linear.x = -0.04
            twist_msg.linear.y = -0.2
            twist_msg.angular.z = -0.1
            self.cmd_vel_pub.publish(twist_msg)

            self.logger.info("[Stop Motion]: Found clear path- Moving RIGHT ")
        
        if (self.right_dist < self.left_dist) and self.front_dist<1.2:
            twist_msg = Twist()

            twist_msg.linear.x = -0.04
            twist_msg.linear.y = 0.2
            twist_msg.angular.z = 0.1
            self.cmd_vel_pub.publish(twist_msg)
            self.logger.info("[Stop Motion]: Found clear path- Moving LEFT")

        if (self.left_dist < 1.2 and self.right_dist < 1.2) and self.front_dist <1.2:
            twist_msg = Twist()

            twist_msg.linear.x = -0.08
            twist_msg.linear.y = 0.
            twist_msg.angular.z = 0.
            self.cmd_vel_pub.publish(twist_msg)

            self.logger.info("[Stop Motion]: Stuck- Moving backwards")
        
        if (self.left_dist>1. and self.right_dist>1.) and self.front_dist>1. and self.point_min_dist>0.2:
            twist_msg = Twist()

            twist_msg.linear.x = 0.07
            twist_msg.linear.y = 0.
            twist_msg.angular.z = 0.
            self.cmd_vel_pub.publish(twist_msg)
            self.logger.info("[Stop Motion]: Front Clear found")
            self.blackboard.set("collision_warning",False)

        '''
        sending self.joy_pub in this method. The frame_id for Joy() message is
        "/dev/input/js0". It is similar to just pressing the deadman button on the joystick.
        Nothing to implement here.
        '''
        ## Uncomment the following lines to publish Joy() message when running on the ROBILE ##
        joyMessage = Joy()
        joyMessage.header.frame_id = "/dev/input/js0"
        joyMessage.axes = [0., 0., 0., 0., 0., 0.]
        joyMessage.buttons = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
        self.joy_pub.publish(joyMessage)

        return None        


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()
    node.run()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

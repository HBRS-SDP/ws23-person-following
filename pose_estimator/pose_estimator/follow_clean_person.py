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
import math
from utils import *

point_min_dist=None
laser= None
depth_at_person= None
person_x= None
last_pos= None
slope_angle= None
ALIGN_THRES=0.05    
min_angle_rad = -1.5700000524520874
max_angle_rad = 1.5700000524520874 
rotation_value=0.50             # Angular veloctiy   
align= 0.0
rotation_direction= 0.0
process_laser= 0.0
dist= 0.0
ROT_THRES=0.45                  # Threshold for angular velocities while following the wall


mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose


def run_online(process_laser,sigma,k):

    '''
    Conversion of Polar to Cartesian coordinates
    '''
    angles = np.linspace(min_angle_rad, max_angle_rad, process_laser.shape[0]) * 180 / np.pi
    # process_laser
    polar_data = np.hstack([process_laser.reshape(-1, 1), angles.reshape(-1, 1)])
    polar_data = np.delete(polar_data, np.where(polar_data[:, 0] == 0), axis=0)
    points = np_polar2rect(reduction_filter(polar_data, sigma, 20, median_filter_size=1, mean_filter_size=2))

    ran_line = online_line_detection(points,e=0.01, incr=0.02, max_dist=2, k=35)
    ran_line = [line for line in ran_line if abs(line[0]) < 4.2]

    if len(ran_line):
        near_line = sorted(ran_line, key=lambda x:x[0])[0]
        d, m,_,_= near_line
        return d, m
    
    # No lines found
    if len(ran_line)==0:
        return 0

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
            self.scan_callback,
            10)
        
        self.safe_min_range = 0.25
        self.person_x = None
        self.depth_at_person = None
        self.point_at_min_dist = 0.
        # Create a subscriber to receive joystick input
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.ranges=None
        self.safe_min_range= 0.25

    def joy_callback(self, joy_msg):
        # Process joystick input here
        # Example: Extract joystick axes and buttons
        self.buttons = joy_msg.buttons

    def scan_callback(self, scan_msg):
        # Use laser scan data for collision avoidance
        self.ranges = scan_msg.ranges
  
        self.run(self.ranges)


    def run(self, ranges):

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
                self.last_pos= None


                laser = np.array(self.ranges)
                laser = np.nan_to_num(laser, nan=0.0)
                laser[laser <= 0.02] = 1.0

                self.dist,self.slope = run_online(process_laser,0.0005,5)
                slope_angle=math.atan(self.slope)
                align=np.deg2rad(90)-abs(math.atan(self.slope))

                self.point_at_min_dist= min(laser)

                if self.point_at_min_dist < self.safe_min_range:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.publisher_.publish(msg)
                    print("Avoiding collision", self.point_at_min_dist)

                else:
                    # Extract landmarks
                    try:
                        landmarks = results.pose_landmarks.landmark

                        left_hip = landmarks[mp_pose.PoseLandmark.LEFT_HIP]
                        right_hip = landmarks[mp_pose.PoseLandmark.RIGHT_HIP]
                        #calculating the centre of the detected person
                        self.person_x = (left_hip.x + right_hip.x) / 2
                        
                        self.last_pos= self.person_x

                        if landmarks:  # Check if landmarks are available
                            print("The center of the found person: ", self.person_x)
                            width, height = image.shape[1], image.shape[0]
                            center_x, center_y = width // 2, height // 2
                            self.depth_at_person = depth_frame.get_distance(center_x, center_y)
                            #Triggering follow motion
                            FollowMotion(self.depth_at_person,self.person_x)

                        if landmarks is False and self.last_pos is not None:
                            #Move to wall
                            if self.blackboard.point_at_min_dist > self.safe_min_range+0.19:
                                rotation_direction= MoveToWall(slope_angle)
                            #Align to wall
                            elif 1.0 > self.blackboard.point_at_min_dist > self.safe_min_range and align > ALIGN_THRES*1.2:
                                AlignToWall(slope_angle,align,rotation_direction,laser,last_pos,point_min_dist,self.dist)
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





class FollowMotion:
    def __init__(self, depth_at_person, person_x, node):
        self.depth_at_person = depth_at_person
        self.person_x = person_x
        self.node = node
        
        # Assuming this is available within the node class
        self.publisher_ = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def setup(self):
        # Setup method implementation if needed
        pass

    def update(self):
        msg = Twist()
        desired_distance = 1.0
        linear_vel = 0.4
        angular_vel = 0.65
        expected_turn = (0.5 - self.person_x) * angular_vel

        print("Expected turn", expected_turn)

        if self.depth_at_person > desired_distance:
            # Move forward
            msg.linear.x = linear_vel
            msg.angular.z = (0.5 - self.person_x) * angular_vel

        elif self.depth_at_person == desired_distance:
            # Stop
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        elif self.depth_at_person < desired_distance:
            # Move backward
            msg.linear.x = -linear_vel

        self.publisher_.publish(msg)


class MoveToWall:
    def __init__(self, slope_angle, node, topic_name="/cmd_vel"):
        self.slope_angle = slope_angle
        self.node = node
        self.cmd_vel_topic = topic_name
    
    def setup(self):
        '''
        Setting up things which generally might require time to prevent delay in tree initialisation
        '''
        self.logger.info("[MOVE_TO_WALL] setting up Move to wall motion behavior")

        # Create publisher to publish rotation commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.cmd_vel_topic,
        )
        self.feedback_message = "setup"
        return True 
   
    def update(self):
        perp_ang = abs(self.slope_angle)

        if perp_ang <= ALIGN_THRES:
            print("Reached near a wall")
            return rotation_direction

        else:            
            if self.slope_angle > 0.0:
                rotation_direction = rotation_value  # Clockwise rotation
            else:
                rotation_direction = -rotation_value  # Counter-clockwise rotation
            max_velocity = 0.30
            angular_velocity = max_velocity
            twist_msg = Twist()
            twist_msg.linear.x = 0.04
            twist_msg.linear.y = 0.
            twist_msg.angular.z = -angular_velocity * rotation_direction
            self.cmd_vel_pub.publish(twist_msg)

            return rotation_direction   

    def terminate(self):
        '''
        terminate() is triggered once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        '''
        self.logger.info("[MOVE_TO_WALL] terminate: publishing zero angular velocity")
        twist_msg = Twist()
        max_velocity = 0.30
        twist_msg.linear.x = max_velocity
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
        
        return None


class AlignToWall:
    def __init__(self, slope_angle, align, rotation_direction, laser, last_pos, point_min_dist, dist, topic_name="/cmd_vel", direction=1):
        self.slope_angle = slope_angle
        self.align_ang = align
        self.rot = rotation_direction
        self.laser = laser
        self.last_pos = last_pos
        self.point_min_dist = point_min_dist
        self.dist = dist
        self.topic_name = topic_name

        # become a behaviour
        super(AlignToWall, self).__init__()

    def setup(self, **kwargs):
        '''
        Setting up things which generally might require time to prevent delay in tree initialisation
        '''
        self.logger.info("[align] setting up aligning behavior")

        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish rotation commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.topic_name,
        )
        self.feedback_message = "setup"
        return True

    def terminate(self, new_status):
        '''
        terminate() is triggered once the execution of the behavior finishes,
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        '''
        self.logger.info("[align] terminate: publishing velocity to align")

        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.

        return None

    def update(self):
        process_laser = np.array(self.laser)

        self.left_dist = np.mean((np.array(process_laser[509:512])))
        self.front_dist = np.mean((np.array(process_laser[253:256])))
        self.right_dist = np.mean((np.array(process_laser[0:3])))

        if self.align_ang <= ALIGN_THRES:
            FollowMotion(self.last_pos, self.rot, process_laser, self.point_min_dist, self.dist, self.align_ang)

        '''
        Closed Corner
        '''
        if self.front_dist > 10.:
            self.front_dist = 999.999
        if self.left_dist > 10.:
            self.left_dist = 999.999
        if self.right_dist > 10.:
            self.right_dist = 999.999

        print("This is front dist:", self.front_dist)
        print("This is left dist:", self.left_dist)
        print("This is right dist:", self.right_dist)

        max_velocity = 0.30

        # Left closed corner
        if (self.left_dist) <= 1.4 and self.front_dist <= 0.9 and self.left_dist < self.right_dist:
            self.logger.info("[align]: CLOSED CORNER IDENTIFIED: Taking right turn, wall is on left")
            print("Left Distance threshold: ", self.left_dist)
            twist_msg1 = Twist()
            twist_msg1.linear.x = 0.
            twist_msg1.linear.y = 0.
            twist_msg1.angular.z = -max_velocity
            self.cmd_vel_pub.publish(twist_msg1)
            return None

        # Right closed corner
        if (self.right_dist) <= 1.4 and self.front_dist <= 0.9 and self.left_dist > self.right_dist:
            self.logger.info("[align]: CLOSED CORNER IDENTIFIED: Taking left turn, wall is on right")
            print("Right Distance threshold: ", self.right_dist)
            twist_msg1 = Twist()
            twist_msg1.linear.x = 0.
            twist_msg1.linear.y = 0.
            twist_msg1.angular.z = max_velocity
            self.cmd_vel_pub.publish(twist_msg1)
            return None

        # Small space
        if (self.right_dist <= 0.8 or self.left_dist <= 0.8) and self.front_dist <= 0.8:
            self.logger.info("[align]: CRAMPED CORNER IDENTIFIED: Moving Backwards")
            print("Left Distance threshold: ", self.left_dist)
            print("Right Distance threshold: ", self.right_dist)
            twist_msg1 = Twist()
            twist_msg1.linear.x = -max_velocity
            twist_msg1.linear.y = 0.
            twist_msg1.angular.z = 0.
            self.cmd_vel_pub.publish(twist_msg1)
            return None

        else:
            rotation_direction = 1
            self.logger.info("[align]: Rotating ROBILE to align to wall")
            if self.slope_angle > 0.0 and (self.left_dist > 0.99 or self.right_dist > 0.99):
                rotation_direction = rotation_value  # Clockwise rotation
            if self.slope_angle < 0.0 and (self.left_dist > 0.99 or self.right_dist > 0.99):
                rotation_direction = -rotation_value  # Counter-clockwise rotation

            angular_velocity = max_velocity
            twist_msg = Twist()
            twist_msg.linear.x = 0.
            twist_msg.linear.y = 0.
            twist_msg.angular.z = angular_velocity * rotation_direction
            self.blackboard.set("rotation_direction", rotation_direction)

            self.cmd_vel_pub.publish(twist_msg)

            return None


class FollowWall:
    def __init__(self, last_pos, rotation_direction, process_laser, point_min_dist, dist, align, topic_name1="/cmd_vel"):
        # Set up topic name to publish rotation commands
        self.topic_name = topic_name1
        self.last_pos = last_pos
        self.rot = rotation_direction
        self.process_laser = process_laser
        self.point_at_min_dist = point_min_dist
        self.dist = dist
        self.align = align

        # Become a behavior
        super(FollowWall, self).__init__()

    def setup(self, **kwargs):
        '''
        Setting up things which generally might require time to prevent delay in tree initialization
        '''
        self.logger.info("[follow] setting up following behavior")

        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.topic_name,
        )
        self.feedback_message = "setup"
        return True

    def terminate(self):
        '''
        terminate() is triggered once the execution of the behavior finishes,
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        '''
        self.logger.info("[follow] terminate: publishing velocity to terminate")

        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
        self.cmd_vel_pub.publish(twist_msg)

        return None

    def update(self):
        self.left_dist = np.mean((np.array(self.process_laser[509:512])))
        self.front_dist = np.mean((np.array(self.process_laser[253:256])))
        self.right_dist = np.mean((np.array(self.process_laser[0:3])))

        # If distance exceeds 15, replace it with 999.999
        if self.front_dist > 15.:
            self.front_dist = 999.999
        if self.left_dist > 15.:
            self.left_dist = 999.999
        if self.right_dist > 15.:
            self.right_dist = 999.999

        print("This is front dist:", self.front_dist)
        print("This is left dist:", self.left_dist)
        print("This is right dist:", self.right_dist)
        print("Rotation Direction", self.rot)

        perp_dist = self.dist
        align_ang = self.align
        point_at_min_dist = self.point_at_min_dist
        print("perp_dist-", perp_dist)
        print("point at min dist: ", point_at_min_dist)

        max_velocity = 0.30

        # Update rotation direction based on last position
        if self.last_pos < 0.4:
            self.rot = -self.rot
        if self.last_pos > 0.6:
            self.rot = self.rot

        # Check conditions for different movements
        if point_at_min_dist < 0.4 and perp_dist < 1.0:
            self.logger.info("[follow]: EXECUTION DONE")
            return None
        elif self.left_dist > 1.2 and self.right_dist > 1.2 and self.front_dist > 1.2:
            twist_msg = Twist()
            twist_msg.linear.x = -0.0
            twist_msg.linear.y = self.rot * 0.20
            twist_msg.angular.z = -self.rot * 1.1
            self.cmd_vel_pub.publish(twist_msg)
            self.logger.info("[follow]: OPEN CORNER FOUND: Now taking a Turn")
            return None
        elif point_at_min_dist > 0.65:
            # Move closer to the wall
            self.logger.info("[follow]: Moving closer to the wall")
            twist_msg = Twist()
            twist_msg.linear.x = max_velocity * 0.45
            twist_msg.linear.y = 0.
            twist_msg.angular.z = -self.rot * ROT_THRES
            self.cmd_vel_pub.publish(twist_msg)
            return None
        elif point_at_min_dist < 0.55:
            # Move away from the wall
            self.logger.info("[follow]: Moving away from the wall")
            twist_msg = Twist()
            twist_msg.linear.x = max_velocity * 0.45
            twist_msg.linear.y = 0.
            twist_msg.angular.z = self.rot * ROT_THRES
            self.cmd_vel_pub.publish(twist_msg)
            return None
        else:
            # Try to be parallel to the wall
            self.logger.info("[follow]: Trying to be Parallel to wall")
            twist_msg = Twist()
            twist_msg.linear.x = max_velocity
            twist_msg.linear.y = 0.
            twist_msg.angular.z = 0.
            self.cmd_vel_pub.publish(twist_msg)
            return None



def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()
    node.run()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

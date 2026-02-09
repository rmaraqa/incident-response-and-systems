#!/usr/bin/env python3

import rospy
import numpy as np
import sys, cv2, cv_bridge, math, os, moveit_commander, moveit_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
import std_msgs.msg

class perception(object):
    def __init__(self):
        rospy.init_node("perception")
        self.grid_subscriber = rospy.Subscriber('grid_topic', Int32MultiArray, self.grid_callback)
        # set up ROS
        self.bridge = cv_bridge.CvBridge()

        # subscribe to the robot's RGB camera data stream
        self.last_image = None
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.front_distance = 3.5
        moveit_commander.roscpp_initialize(sys.argv)
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.something_in_hand = False
        self.first_ball = True  # Flag to control the first ball logic
        # Manually defining the entire grid matrix as seen in the image
        #self.grid = np.array([
        #    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #    [0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 4, 4, 0, 0, 0, 0, 2, 0],
        #    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 4, 0, 0, 0, 0, 0, 0],
        #    [0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 4, 4, 0, 0, 0, 0, 0, 0],
        #    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #    [0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0],
        #    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        #])
        
        self.robot_position = None  # This needs to be updated with actual data
        self.basket_position = (0, 0)  # Assuming the basket is at (0,0)

        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm and gripper position
        self.move_group_arm.go([0, -1, 0.9, 0], wait=True)
        rospy.sleep(2)
        self.move_group_gripper.go([0.01, 0.01], wait=True)
        rospy.sleep(5)
        print(f"Set Up Done")

    def grid_callback(self, msg):
        self.grid = np.array(msg.data).reshape((10, 20))

    def scan_callback(self, msg):
        """
        Processes LiDAR data to update the closest distance in front of the robot.
        """
        if len(msg.ranges) < 400:
            processed_ranges = [r if r != 0.0 else 3.5 for r in msg.ranges]
            front_ranges = processed_ranges[:10] + processed_ranges[-10:]
        else:
            processed_ranges = [r if r != math.inf else 12 for r in msg.ranges]
            front_index = (len(processed_ranges)-1) // 2
            front_ranges = processed_ranges[front_index-10:front_index+10]
        # Calculate min distance 
        self.front_distance = min(front_ranges)
        
    @staticmethod
    def point_to_line_distance(px, py, A, B, C):
        """Function to calculate distance from point (px, py) to the line Ax + By + C = 0"""
        return abs(A * px + B * py + C) / np.sqrt(A**2 + B**2)

    def find_robot_position(self, grid):
        """Find the robot's position in the grid by calculating the centroid of all cells marked with '4'."""
        robot_positions = np.argwhere(grid == 4)
        if len(robot_positions) == 0:
            print("No robot detected in the grid.")
            return None  # No robot found in the grid
        # Calculate the centroid of the robot positions
        centroid = np.mean(robot_positions, axis=0).astype(int)
        return tuple(centroid)  # Returns a tuple (row_index, col_index)

    @staticmethod
    def find_ball_closest_to_diagonal(grid, start, end):
        """Find the ball closest to the diagonal line from start to end."""
        balls = np.argwhere(grid == 2)
        if len(balls) == 0:
            return None
        x1, y1 = start
        x2, y2 = end
        A = y2 - y1
        B = -(x2 - x1)
        C = x2 * y1 - y2 * x1
        closest_ball = min(balls, key=lambda ball: perception.point_to_line_distance(ball[1], ball[0], A, B, C))
        return tuple(closest_ball)
        
    def image_callback(self, msg):
        """
        Processes camera images to guide robot actions based on visual cues.
        """
        # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        if image is None or image.size == 0:
            print("Failed to convert image or image is empty")
            return
        h, w, d = image.shape
        self.last_image = image
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # checking to see if there is something in the range
        if not self.something_in_hand:
            if self.first_ball:
                print("Looking for the first ball...")
                self.robot_position = self.find_robot_position(self.grid)  # Implement this method based on your setup
                print(f"Robot position: {self.robot_position}")
                closest_ball = self.find_ball_closest_to_diagonal(self.grid, self.robot_position, self.basket_position)
                if closest_ball:
                    print(f"First ball chosen at position {closest_ball}")
                    self.go_to_first_ball(closest_ball)
                    self.first_ball = False
            else:
                # Standard method for subsequent balls
                print("Looking for subsequent balls...")
                self.process_standard_ball_detection(hsv)
        else:
            # Handling for dropping the ball into the basket
            print("Going to basket...")
            self.handle_basket_interaction(hsv)

    def process_standard_ball_detection(self, hsv):
        """
        Standard processing to detect and approach balls using HSV thresholding.
        """
        h, w, _ = self.last_image.shape
        current_range = (np.array([125, 50, 50]), np.array([145, 255, 255]))
        lower = current_range[0]
        upper = current_range[1]
        mask = cv2.inRange(hsv, lower, upper)
        search_left = int(1 * w / 4)
        search_right = int(3 * w / 4)
        mask[0:h, :search_left] = 0
        mask[0:h, search_right:] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            print(f"Ball found at ({cx}, {cy})")
            cv2.circle(self.last_image, (cx, cy), 20, (0, 0, 255), -1)
            cx_error = cx - w / 2
            self.go_to(cx_error)
        else:
            self.spin_around()

    def handle_basket_interaction(self, hsv):
        """
        Handling for detecting the basket and dropping the ball.
        """
        h, w, _ = self.last_image.shape
        current_range = (np.array([0, 0, 0]), np.array([180, 255, 30]))  # BLACK COLOR RANGE for the basket
        lower = current_range[0]
        upper = current_range[1]
        mask = cv2.inRange(hsv, lower, upper)
        search_left = int(1 * w / 4)
        search_right = int(3 * w / 4)
        mask[0:h, :search_left] = 0
        mask[0:h, search_right:] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            print(f"Basket found at ({cx}, {cy})")
            cv2.circle(self.last_image, (cx, cy), 20, (0, 0, 255), -1)
            cx_error = cx - w / 2
            self.go_to(cx_error)
        else:
            self.spin_around()

    def go_to_first_ball(self, ball_pos):
        # Assume self.robot_position is already defined as (x_r, y_r)
        print(f"Going to first ball at position {ball_pos} from robot position {self.robot_position}")
        x_r, y_r = self.robot_position
        x_b, y_b = ball_pos

        # Calculate direction vector from robot to ball
        direction_vector = np.array([x_b - x_r, y_b - y_r])
        # Calculate the angle to turn in radians
        angle_to_turn = np.arctan2(direction_vector[1], direction_vector[0])

        # This angle_to_turn is now in radians which might need to be adjusted if your robot
        # needs the angle in a different range or format
        # Ensure the robot turns the shortest direction
        current_orientation = 0  # Assuming robot is initially facing 'north' and this is updated during navigation
        angle_to_turn = (angle_to_turn - current_orientation + np.pi) % (2 * np.pi) - np.pi

        # If the angle to turn is significant
        if abs(angle_to_turn) > 0.1:  # Adjust this threshold based on your robot's precision
            self.send_movement(0, angle_to_turn)
        else:
            # If aligned, decide to move towards the object
            if self.front_distance > 0.25 and not self.something_in_hand:
                self.send_movement(min(0.1, 0.1 * self.front_distance), 0)  # Move straight towards the ball
                #print(f"Approaching ball... Front Distance: {self.front_distance}")
            elif self.front_distance > 0.5 and self.something_in_hand:
                self.send_movement(min(0.1, 0.1 * self.front_distance), 0)  # Continue moving towards the drop-off point
            else:
                # Stop moving if close enough to pick up or drop the object
                self.send_movement(0, 0)
                print ("at the first ball")
                #print(f"Close to object... Front Distance: {self.front_distance}")
                if not self.something_in_hand:
                    self.pick_object()
                    print ("picking up the first ball")
                else:
                    self.drop_object()
                    self.something_in_hand = False
                    self.send_movement(-0.5, 0)  # Back up a bit after dropping
                    rospy.sleep(2)
    
    def go_to(self, px_error):
        # Convert pixel error to angular error
        angular = -1 * (px_error / 100)
        # if not aligned
        if abs(angular) > 1.0:
            self.send_movement(0, angular)
        # is aligned
        else:
            if self.front_distance > 0.25 and not self.something_in_hand:
                self.send_movement(min(0.1, 0.1 * self.front_distance), angular)
            elif self.front_distance > 0.5 and self.something_in_hand:
                self.send_movement(min(0.1, 0.1 * self.front_distance), angular)
            else:
                self.send_movement(0, 0)
                print ("at the ball")
                if not self.something_in_hand:
                    self.pick_object()
                    print ("picking the ball")
                else:
                    self.drop_object()
                    self.something_in_hand = False
                    self.send_movement(-0.5, 0)
                    rospy.sleep(2)

    def move_arm(self, pos_name, wait=7):
        print(f'Moving arm: {pos_name}')
        # default pos
        move = [0, -1, 0.9, 0]
        if pos_name == 'down':
            move = [0, math.radians(73), math.radians(-27), math.radians(-12)]
        elif pos_name == 'up':
            move = [0, -0.5, -0.5, 0]
        self.move_group_arm.go(move, wait=True)
        rospy.sleep(wait)

    def grip(self, wait=7):
        self.move_group_gripper.go([0.005, 0.005])
        rospy.sleep(wait)

    def ungrip(self, wait=7):
        self.move_group_gripper.go([0.01, 0.01])
        rospy.sleep(wait)

    def pick_object(self):
        if self.something_in_hand:
            return
        self.ungrip()
        self.move_arm('down')
        self.grip()
        self.move_arm('up')
        self.something_in_hand = True
        rospy.sleep(1)

    def drop_object(self):
        print('Dropping object')
        self.move_arm('down')
        self.ungrip()
        self.move_arm('reset')

    def send_movement(self, velocity, angular):
        move_cmd = Twist()
        move_cmd.linear.x = velocity
        move_cmd.angular.z = angular
        # send velocity command
        self.velocity_publisher.publish(move_cmd)
        rospy.sleep(0.05)

    def spin_around (self):
        self.send_movement(0,0.1)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = perception()
        rospy.sleep(1)
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        print(f"Done")

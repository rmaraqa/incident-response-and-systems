#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
from std_msgs.msg import Int32MultiArray
import time

def create_environment_grid(depth_image, color_image, grid_size=(10, 20)):
    height, width = depth_image.shape
    cell_h, cell_w = height // grid_size[0], width // grid_size[1]
    environment_grid = np.zeros(grid_size, dtype=int)


    # Define thresholds
    wall_threshold = 3000  # Adjust this threshold based on actual measurements of wall distances
    floor_threshold = 3000  # Example threshold for floor (if needed)
    purple_lower = np.array([125, 50, 50])  # Adjusted to better capture purple
    purple_upper = np.array([145, 255, 255])
    black_lower = np.array([0, 0, 0])
    black_upper = np.array([180, 255, 30])


    for i in range(grid_size[0]):
        for j in range(grid_size[1]):
            # Segment grid cell
            cell_depth = depth_image[i * cell_h:(i + 1) * cell_h, j * cell_w:(j + 1) * cell_w]
            cell_color = color_image[i * cell_h:(i + 1) * cell_h, j * cell_w:(j + 1) * cell_w]


            # Ignore floor
            if np.all(cell_depth < floor_threshold):
                continue


            # Check for walls
            if np.any(cell_depth > wall_threshold):
                environment_grid[i, j] = 0


            # Check for purple balls
            hsv_cell = cv2.cvtColor(cell_color, cv2.COLOR_BGR2HSV)
            purple_mask = cv2.inRange(hsv_cell, purple_lower, purple_upper)
            if np.any(purple_mask):
                environment_grid[i, j] = 2


            # Check for black objects (robots) using color
            black_mask = cv2.inRange(hsv_cell, black_lower, black_upper)
            if np.any(black_mask):
                environment_grid[i, j] = 4


    return environment_grid

def get_environment_grid():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    roi_x = 0
    roi_y = 0
    roi_width = 1000
    roi_height = 250

    try:
        while True:
            #time.sleep(5)
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply cropping
            depth_image_cropped = depth_image[roi_y:roi_y+roi_height, roi_x:roi_x+roi_width]
            color_image_cropped = color_image[roi_y:roi_y+roi_height, roi_x:roi_x+roi_width]

            # Create environment grid
            grid = create_environment_grid(depth_image_cropped, color_image_cropped)
            print (grid)

            # Visualization (optional)
            cv2.imshow('RealSense Cropped', color_image_cropped)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
    
def publish_grid(grid):
    pub = rospy.Publisher('grid_topic', Int32MultiArray, queue_size=10)
    rospy.init_node('grid_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz
    msg = Int32MultiArray(data=grid.flatten())
    pub.publish(msg)
    rate.sleep()
if __name__ == "__main__":
    grid = get_environment_grid()
    print(grid)  # For testing- print the grid once.
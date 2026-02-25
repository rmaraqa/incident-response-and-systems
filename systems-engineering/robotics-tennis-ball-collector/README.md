# Tennis Ball Collector

## Project Description

The goal of this project was to create a  turtlebot that collects tennis balls after a tennis match as efficiently as possible. This project required calculating and testing to determine which algorithm was most efficient: the one that collected the most balls over the least distance. Since the speed is uniform, distance is directly proportional to time, meaning that as the distance covered increases, the time taken increases.

The robot collects all the tennis balls using the algorithm and places them in a basket for collection before the start of the next round of padel. The RealSense camera is used to create a god mode effect, producing a 2D grid indicating the locations of all the tennis balls and the turtlebot. This is useful for getting our turtlebot to navigate to the first ball in our algorithm, as that is the most important calculation. The camera is set up at a high angle above our environment and detects the tennis balls and the turtlebot. This god mode feature helps better understand the environment than the turtlebot can using its camera or LiDAR sensors. This method ensured maximum accuracy and minimized the room for error. 


https://github.com/user-attachments/assets/bdff3d59-bc11-47f7-b312-eabc5037e4ce


https://github.com/user-attachments/assets/6bda20ee-a9af-4789-b631-4bf7ff07e377

The other main component of the project is the algorithm. Briefly, the algorithm detects the tennis ball closest to the diagonal line between the turtlebot and the basket, then collects the tennis balls and the ball closest to that line. Once it drops the ball off, it implements a greedy algorithm that collects the nearest tennis ball to the turtlebot (now at the basket), picks it up, returns to the basket, and repeats this until all tennis balls are removed. 

The combination of these two components helped make the algorithm efficient, each controlling one aspect. Together, these two components enable two different methods for collecting the balls between the first ball and the remaining balls.

## System Architecture

### Robotics Algorithm

The primary goal t is to efficiently collect tennis balls after a tennis match using a TurtleBot equipped with a RealSense camera and LiDAR. The algorithm is designed to minimize the distance the robot travels, thereby reducing the time required to collect all the balls. 

The algorithm can be divided into two main phases:
1. **First Ball Collection**: 
   - The robot identifies its initial position and calculates the ball closest to the diagonal line from its starting position to the basket.
   - This phase is implemented in the `go_to_first_ball` method of the `perception` class, which uses a combination of grid-based positioning and vision processing.
   -
   - <img width="955" height="476" alt="3" src="https://github.com/user-attachments/assets/3f07be3f-9ba3-418b-bd11-038acbac20e8" />



2. **Subsequent Ball Collection**:
   - After collecting the first ball, the robot switches to a greedy algorithm to collect the nearest ball to its current position (now at the basket) and continues this process until all balls are collected.
   - This phase is implemented in the `process_standard_ball_detection` and `handle_basket_interaction` methods, which handle detection and movement towards the balls and the basket.

### Major Components

1. **Perception**:
   - The `perception` class is responsible for processing camera and LiDAR data to identify and locate tennis balls.
   - Key functions include `image_callback`, `scan_callback`, and `find_robot_position`.
   - The robot's movements are controlled through methods like `go_to_first_ball`, `go_to`, `pick_object`, and `drop_object`.

2. **Environment Grid Creation**:
   - Implemented in the `Camera.py` script, this component uses the RealSense camera to create a grid representation of the environment.
   - The `create_environment_grid` function processes depth and color images to identify walls, floors, balls, and robots.
   - The `publish_grid` function publishes the grid data to a ROS topic for use by the `perception` node.
![4](https://github.com/user-attachments/assets/3e0bedd5-a86f-4b08-b6db-c7ddad29f64c)
   


3. **ROS Communication**:
   - ROS nodes and topics facilitate communication between different components.
   - The `perception` node subscribes to camera and LiDAR topics and publishes movement commands.
   - The `grid_publisher` node publishes the environment grid to the `grid_topic`.

## ROS Node Diagram

![5](https://github.com/user-attachments/assets/28bde40e-1cc9-4ba8-b69e-08f1fcefbf9b)


- `Camera.py` node:
  - Publishes to `grid_topic`.
- `Perception.py` node:
  - Subscribes to `camera/rgb/image_raw`, `/scan`, `grid_topic`.
  - Publishes to `cmd_vel`.

### Execution
Here is a guide on how to run the code and make sure it works:

Terminal 1: 
$ roscore

- For Camera:
Terminal 2: 
rosrun final_project_rm camera.py

- For Perception:
Terminal 3: 
$ ssh pi@IP_OF_TURTLEBOT
$ set_ip LAST_THREE_DIGITS
$ bringup

Terminal 4:
$ ssh pi@IP_OF_TURTLEBOT
$ set_ip LAST_THREE_DIGITS
$ bringup_cam

Terminal 5:
roslaunch final_project_rm action.launch

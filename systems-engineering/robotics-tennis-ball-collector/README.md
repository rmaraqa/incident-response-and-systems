# final_project_rm

## Project Description

The goal of our project is to create a turtlebot that collects tennis balls after a padel match as efficiently as possible. This is interesting because before even starting on the project, we had to calculate and test to decide which algorithm was the most efficient and collected the most balls covering the least distance, as since the speed is uniform, distance was directly proportional to time, meaning that as the distance covered increased, the time taken increased.

By the end of our project, we were able to have the robot collect all the tennis balls using our algorithm and place them in a basket for collection before the start of the next round of padel. The main components of our project were the use of a RealSense camera to create a god mode effect, which produced a grid that indicated the locations of all the tennis balls and the turtlebot in a 2D array. This is useful for getting our turtlebot to navigate to the first ball in our algorithm, as that is the most important calculation. The camera is set up at a high angle above our environment and detects the tennis balls and the turtlebot. This god mode feature helps us better understand the environment than the turtlebot can using its camera or LiDAR sensors. This method ensured maximum accuracy and minimized the room for error. 


https://github.com/user-attachments/assets/bdff3d59-bc11-47f7-b312-eabc5037e4ce


https://github.com/user-attachments/assets/6bda20ee-a9af-4789-b631-4bf7ff07e377

The other main component of the project (which will be expanded on in the next section) is our algorithm. Briefly, the algorithm detects the tennis ball nearest the diagonal from the turtlebot to the basket, then collects the tennis balls and the ball closest to that line. Once it drops the ball off, we implement a greedy algorithm that collects the nearest tennis ball to the turtlebot (now at the basket), picks it up, returns to the basket, and repeats this until all tennis balls are removed. 

The combination of these two components helped us implement the algorithm that we argue is the most efficient, by each controlling one aspect of our algorithm. Together, these two components enable us to use a different method for collecting the balls between the first ball and the rest.

## System Architecture

### Robotics Algorithm

The primary goal of our project is to efficiently collect tennis balls after a padel match using a TurtleBot equipped with a RealSense camera and LiDAR. The algorithm is designed to minimize the distance the robot travels, thereby reducing the time required to collect all the balls. 

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
   - 

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

## Challenges

One of the main challenges we faced was accurately detecting and distinguishing tennis balls from other objects in the environment. This was initially difficult due to varying lighting conditions and the similarity in color between the balls and some parts of the environment. We overcame this by fine-tuning our HSV color thresholds and implementing additional checks using depth data from the RealSense camera to improve accuracy. Despite making these changes, we still had some color issues, so we bought purple tennis balls to improve accuracy.

![6](https://github.com/user-attachments/assets/1143a1a4-0d12-44ec-80c0-0ce7d232220f)


Another challenge was learning to work with Realsense, as it wasn't something we had learned through the quarter; instead, we learned it from YouTube videos and the TA's help setting it up and getting it working. Oftentimes, it was very buggy, and the laptop we had to use for the camera wouldn't work. It took trial and error to get it working, and we also went to the second CSIL opening to avoid latency issues.

![7](https://github.com/user-attachments/assets/72cb7afa-eff7-4b76-a8f1-ffbcff16d461)



## Future Work

Given more time, we would improve our implementation by integrating a more sophisticated SLAM (Simultaneous Localization and Mapping) algorithm to enhance the robot's navigation capabilities. Additionally, we would refine the grid-based environment mapping to dynamically update the robot's position and detected objects in real time, enabling more adaptive and efficient path planning. Another improvement would be to find a way to collect multiple balls before heading to the basket, as this would create a window of opportunity for more efficient algorithms to collect them in even less time. 

## Takeaways

- **Effective Team Collaboration**:
  - Working in pairs taught us the importance of clear communication and task delegation. By dividing the project into distinct components (e.g., perception, navigation, RealSense, and ROS communication), we were able to work more efficiently and integrate our work seamlessly.
- **Robust Sensor Integration**:
  - Integrating multiple sensors (camera and LiDAR) significantly improved the robot's perception and decision-making capabilities. This experience highlighted the importance of sensor fusion in robotics to enhance accuracy and reliability in real-world applications.
- **Future in Robotics**:
  - Expanding on the last point, this project has shown us that robotics has beautiful real-world applications that aim to increase the efficiency of activities and complete tasks with more accuracy. All the groups are completing very interesting projects that I could see myself using, whether to make my life easier or just for entertainment purposes

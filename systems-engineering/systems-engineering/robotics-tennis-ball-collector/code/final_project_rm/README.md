# final_project_rm

## Project Description

The goal of our project is to create a turtlebot that collects tennis balls after a padel match in the most efficient manner. This is interesting because before even starting on the project, we had to calculate and test to decide which algorithm was the most efficient and collected the most balls covering the least distance, as since the speed is uniform, distance was directly proportional to time, meaning that as the distance covered increased, the time taken increased.

By the end of our project, we were able to make the robot collect all the tennis balls using our algorithm and placing them in a basket, to be collected before the start of the next round of padel. The main components of our project were the use of a RealSense camera to create a god mode effect in which it created a grid that pointed out the location of all the tennis balls and the location of the turtlebot in a 2D array. This is useful in getting our turtlebot to navigate to the first ball in our algorithm as that is the most important calculation of the algorithm. The camera is set up at a high angle above our environment and detecs the tennis balls and turtlebot. This god mode feature aids us in knowing the environment better than what the turtlebot can do using its camera or LiDAR sensors. This method ensured maximum accuracy and minimized the room for error. 


![IMG_9474-ezgif com-video-to-gif-converter](https://github.com/Intro-Robotics-UChicago-Spring-2024/final_project_rm/assets/114775053/b6df1e1d-ba0d-4fb9-a4ea-971eb5059182)
![IMG_9478-ezgif com-video-to-gif-converter](https://github.com/Intro-Robotics-UChicago-Spring-2024/final_project_rm/assets/114775053/4d701379-1d94-4d24-8be7-5e802bb4c921)



The other main component of the project (which will be expanded on in the next section) is our algorithm. To explain briefly, our algorithm detects the tennis ball nearest to the diagonal from the turtlebot to the basket collecting the tennis balls and collects the tennis ball closest to that line. Once it drops that ball off, we implement a greedy algorithm which collects the nearest tennis ball to the turtlebot (whose location is now at the basket) and then goes to the tennis ball, picks it up, then goes back, and repeats this until all tennis balls are removed. 

The combination of these two components helped us implement the algorithm that we argue is the most efficient by each controlling one aspect of our alogrithm. Combined, these two components enables us to use a different method of collecting the balls between the first ball and the rest of them.

## System Architecture

### Robotics Algorithm

The primary goal of our project is to efficiently collect tennis balls after a padel match using a TurtleBot equipped with a RealSense camera and LiDAR. The algorithm is designed to minimize the distance traveled by the robot, thereby reducing the time taken to collect all the balls. 

The algorithm can be divided into two main phases:
1. **First Ball Collection**: 
   - The robot identifies its initial position and calculates the ball closest to the diagonal line from its starting position to the basket.
   - This phase is implemented in the `go_to_first_ball` method of the `perception` class, which uses a combination of grid-based positioning and vision processing.
   -
   - <img width="741" alt="output" src="https://github.com/Intro-Robotics-UChicago-Spring-2024/final_project_rm/assets/114775053/bf7d8c91-2768-4a84-a534-faed70986f83">


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
   - <img width="1056" alt="GRID" src="https://github.com/Intro-Robotics-UChicago-Spring-2024/final_project_rm/assets/114775053/d597ce04-5524-4591-8a63-0989a4731411">


3. **ROS Communication**:
   - ROS nodes and topics facilitate communication between different components.
   - The `perception` node subscribes to camera and LiDAR topics and publishes movement commands.
   - The `grid_publisher` node publishes the environment grid to the `grid_topic`.

## ROS Node Diagram

<img width="600" alt="Screenshot 2024-05-23 at 12 38 45 PM" src="https://github.com/Intro-Robotics-UChicago-Spring-2024/final_project_rm/assets/91807696/189a504a-ae68-4117-be30-e896fb8acc3b">

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

One of the main challenges we faced was accurately detecting and differentiating between tennis balls and other objects in the environment. This was initially difficult due to varying lighting conditions and the similarity in color between the balls and some parts of the environment. We overcame this by fine-tuning our HSV color thresholds and implementing additional checks using depth data from the RealSense camera to improve accuracy. However, despite making these changes, we still faced some difficulties with the color and so we bought purple tennis balls to ensure better accuracy.

<img width="552" alt="Screenshot 2024-05-16 at 12 51 20 PM" src="https://github.com/Intro-Robotics-UChicago-Spring-2024/final_project_rm/assets/114775053/dde6558f-e238-4835-9eac-8dddd7c260b4">

Another challenge was learning to work with Realsense as it wasn't something we had learned through the quarter but rather we learned it using Youtube videos and the TA's help to set it up and get it working. Oftentimes it was very buggy and the laptop we had to use for the camera wouldn't work but it took trial and error to get it working and also going the second CSIL opens to avoid latency issues.

<img width="979" alt="Screenshot 2024-05-23 at 4 00 38 PM" src="https://github.com/Intro-Robotics-UChicago-Spring-2024/final_project_rm/assets/114775053/373b11a2-0c48-410b-99f0-da5e9c2234bb">


## Future Work

Given more time, we would improve our implementation by integrating a more sophisticated SLAM (Simultaneous Localization and Mapping) algorithm to enhance the robot's navigation capabilities. Additionally, we would refine the grid-based environment mapping to dynamically update the robot's position and detected objects in real-time, allowing for more adaptive and efficient path planning. Another improvement would be thinking of a way to collect multiple balls before heading to the basket as that would create the window of opportunity for greater and more fficient algorithms that would collect the balls in even less time. 

## Takeaways

- **Effective Team Collaboration**:
  - Working in pairs taught us the importance of clear communication and task delegation. By dividing the project into distinct components (e.g., perception, navigation, realsense, and ROS communication), we were able to work more efficiently and integrate our work seamlessly.
- **Robust Sensor Integration**:
  - Integrating multiple sensors (camera and LiDAR) significantly improved the robot's perception and decision-making capabilities. This experience highlighted the importance of sensor fusion in robotics to enhance accuracy and reliability in real-world applications.
- **Future in Robotics**:
  - Expanding on the last point, this project has shown us that robotics has beautiful real-world application that aim to increase the efficiency of activities and completing tasks with more accuracy. All the groups are completing very interesting projects that I could see myself using, whether to make my life easier or just for entertainment purposes

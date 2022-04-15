# CE215- University of Essex -Assignments

This was carried out in the Robotics Operating System 2 (ROS2) environment using the Gazebo and also tested on the Turtlebot3, more specifically the burger model. 
- More information about ROS2: https://www.ros.org
- More information on Turtlebot3: https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
  
# Details about the task
These are robotics tasks I did as part of my University Module. There were two main tasks
- Trajectory Tracking, which makes the robot move in a square using a closed loop control system such as PID
- Command the robot to follow set waypoints in a map, whilst having simple obstacle detection and avoidance using the robots Laser Scanner
    - The robot also will generate a map (2D Occupancy Grid Map) by running the cartographer node in ROS2. Which uses the SLAM Algorithm.

For both of these tasks, I programmed a Proportional, Integral, Differential (PID) Control Algorithm.

# Overview of the codes
- **waypont_follow.py**: Code for the second task. Move to waypoints in an area/room using PID and avoid obstacles using the laser scanner 
- **my_node.py**: Code for the first task, i.e., to track a square trajectory using PID
- **random_move.py**: Code to allow the robot to move randomly in a free space while avoiding obstacles
- pv_ndde.py: A test code, to understand publishing velocity in ROS2
- hw_node.py: Just a simple test node, prints hello world to terminal
- sl_node.py: A test code, to understand subscribing to laser data in ROS2
- so_node.py: A test code, to understand subscribing to robot odometry data in ROS2
- test_node.py: A test code to test any relevant ideas I had
# The task

This section describes the task this repo's code is to solve 

## Task description

Suppose you are developing path planning module of the robot. You will receive a test map files, both yaml and pgm. The goal for this task is to implement and test on the provided map global path planning algorithm, that will take into account obstacles on the map and will be capable of constructing feasible route from point A to point B.

## Rules

1. Deliverable is a ROS/ROS2 package, that is capable of constructing a route from point A to point B, avoiding obstacles from the map.
2. Assume that the robot has circular footprint of diameter 160 mm.
3. You should subscribe to

   * /start - start position of the route, geometry_msgs/PoseStamped
   * /end   - end position of the route, geometry_msgs/PoseStamped
    
   Route should be constructed from last received poses on topics above

4. You should publish found route to
   * /path - found path, nav_msgs/Path.
  
5. The solution must contain C++ or Python code.
6. You are not allowed to use 3rd party packages or simple algorithms like A*, Djisktra for path planning.

# Solution

## Resources

List of assets used to complete the task:  

1. Win10 2004 + Oracle VMBox + Ubuntu 20.04
2. ROS2 Foxy  
   * colcon  
   * rviz  
   * nav_msgs  
   * geometry_msgs  
   * map_server  
3. rqt and rqt_graph
4. VSCode + Python 3.8.5 64-bit  
   * autopep8
   
## Assumptions 

1. the robot has ~~160 mm~~ 200 mm diameter (4 pixel wide)  
>>>Given the map resolution (50mm per pixel), diameter is rounded up from 160 to 200 mm  

   * the pixelized footprint as follows:
   
    ----oo----  
    ---oooo---  
    ---oooo---  
    ----oo----  

2. the robot operates at 100 Hz
>>>two closest stamps in the Path msg may occur only with 0.01 s delimeter
3. the robot is not omnidirectional
>>>In order to navigate correctly, there should be a yaw angle, with (x,y) and the timestamp
4. Robot's state at any given time consist of these parameters:  
   * (x, y)
   * phi, yaw angle
   * width 

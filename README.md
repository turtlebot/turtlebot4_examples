# turtlebot4_examples
Example applications for the TurtleBot 4.

Check out the [TurtleBot 4 User Manual](https://turtlebot.github.io/turtlebot4-user-manual/) for more information about the TurtleBot 4.

## Python Examples

### FollowBot

This example utilises the OAK-D camera to detect people and follow them. 

To run, call:

```bash
ros2 launch turtlebot4_python_examples followbot.launch.py
```

## C++ Examples

### Mobilenet Spatial Publisher

This example is a modified version of the [Depthai Mobilenet node](https://github.com/luxonis/depthai-ros-examples/blob/main/depthai_examples/ros2_src/mobilenet_publisher.cpp). It implements the Mobilenet Spatial Detection Network which uses the stereo cameras to calculate the distance of detections from the camera.

To run, call:

```bash
ros2 launch turtlebot4_cpp_examples mobilenet_spatial_publisher.launch.py
```
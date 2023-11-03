An opinionated ROS2 C++ node template, optimized for ISC.


# Instructions


1. Clone repo inside your workspaces src directory (Ex. phnx_ws/src)
2. `rosdep install --from-paths . --ignore-src -r -y` to install deps
3. `colcon build` to make sure the repo builds before you mess with it
4. Replace the following in both file names and code exactly and consistently.
   1. hybrid_pp: Replace with the package name. Use snake case. Ex. `data_logger`
   2. PurePursuitNode: Replace with the node name. Use Pascal case. Ex. `DataLogger`
5. `colcon build` again. If it builds, you are done
6. Rename outer folder
7. Review the optional dependencies, and remove what you do not need


# Dependencies
Some common extra dependencies are included. Review them and remove what you don't need.
These are marked with TODO_EXTRA.


# Features


- Unit tests
- ROS-Industrial github CI (will test units and lints)
- C++ formatting via clangformat
- A selection of sane lints
- A single node setup in a multithreaded executor


# File structure


```
.
├── include
│   └── hybrid_pp
│       └── PurePursuitNode_node.hpp
├── package.xml
├── README.md
├── src
│   ├── PurePursuitNode.cpp
│   └── PurePursuitNode_node.cpp
└── tests
    └── unit.cpp
```


PurePursuitNode_node: Source files for the ROS2 node object itself, and only itself


PurePursuitNode.cpp: Source for the main function of the node, and only the main function


tests/unit.cpp: Example file for unit tests. This is linked to the node and ros, so both can be used


# hybrid_pp
From package '[hybrid_pp](https://github.com/ISC-Project-Phoenix/hybrid_pp)'
# File
`./src/hybrid_pp.cpp`


## Summary
 Path tracking implementation of the pure pursuit algorithm used to command velocity and ackerman angle to navigate to calculated points along a selected path. The hybrid section
will come into play with the use of the lidar and a TBD obstacle avoidance algorithm.


### Publishes
- `/nav_ack_vel`:  The calculated ackerman angle and velocity to be set to phoenix.
- `visualization_marker`: The look ahead distance and path arc prediction to be plotted on rviz for live visualization of the path predictions.


### Subscribes
- `/odom_can`: Odometry from the can bus, this is used to get the linear velocity to determine a look ahead distance.
- `/camera/mid/rgb`: Camera data, to be synced with training data.


## Params
- `min_look_ahead_distance`: The minimum look ahead distance to intersect the path and obtain a path point that is kinematically possible for phoenix.
- `max_look_ahead_distance`: The maximum look ahead distance to intersect the path and obtain a path point that is within the max range of the camera.
- `k_dd`: A constant multiplier used to scale the speed in the calculation for look ahead distance.
- `max_speed`:  Max possible speed of phoenix in meters per second.
- `wheel_base`: The wheelbase of phoenix in meters.
- `gravity_constant`: The constant acceleration due to gravity.
- `debug`: Debug flag to determine if we want to publish the visualization markers.
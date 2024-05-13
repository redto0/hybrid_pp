# hybrid_pp

From package '[hybrid_pp](https://github.com/ISC-Project-Phoenix/hybrid_pp)'

# File

`./src/hybrid_pp.cpp`

## Summary

Path tracking implementation of the pure pursuit algorithm used to command velocity and ackerman angle to navigate to
calculated points along a selected path. In addition to standard pure pursuit, we also add additional obstacle avoidance
via a repulsion field-esk algorithm. This works on LiDAR scans, and wraps paths around any obstacle in front of the bot.

### Algorithm

Currently we take in the path from obj_planner and create a spline from the list of points. Then based off our current
speed we calculate the look ahead distance which is used to check the point of intersection infront of us along the
spline. That point of intersection then becomes our target point to which we calculate our steering angle and command
velocity. When paths are first received, we massage each spline point to move away from local scan points, where each
point contributes to the change. This causes the spline to wrap around obstacles.

### Publishes

- `/nav_ack_vel`:  The calculated ackerman angle and velocity to be set to phoenix.
- `visualization_marker`: The look ahead distance, path arc prediction, and interpolated path to be plotted on rviz for
  live visualization of the path predictions.

### Subscribes

- `/odom`: Odometry, this is used to get the linear velocity to determine a look ahead distance.
- `/path`: This is the path that is being supplied from the obj_planner node, should be a list of midpoints between
  respective left and right cones.
- `/scan`: Lidar scans to avoid.

## Params

- `min_look_ahead_distance`: The minimum look ahead distance to intersect the path and obtain a path point that is
  kinematically possible for phoenix.
- `max_look_ahead_distance`: The maximum look ahead distance to intersect the path and obtain a path point that is
  within the max range of the camera.
- `k_dd`: A constant multiplier used to scale the speed in the calculation for look ahead distance.
- `rear_axle_frame`: A string used for the odom to rear_axle transformation.
- `max_speed`:  Max possible speed of phoenix in meters per second.
- `min_speed`:  Min required speed in m/s.
- `wheel_base`: The wheelbase of phoenix in meters.
- `gravity_constant`: The constant acceleration due to gravity.
- `debug`: Debug flag to determine if we want to publish the visualization markers.
- `filter_window`: Size of filter used to dampen commands.
- `stop_on_no_path`: If true, stops the vehicle if no more points in the path are found. Else continues the last
- `avoidance`: If spline points are under this value in meters from obstacles, then that obstacle will nudge the spline
  point away. Smaller values will lead to the kart moving closer to obstacles.
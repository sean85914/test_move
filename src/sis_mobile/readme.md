## How to run
```
 $ roslaunch sis_mobile config.launch mecanum:=false
 $ roslaunch sis_mobile move_base.launch mecanum:=false
```
Set mecanum to true if you use mecanum mobile robot
## Usage example
After you launch two files above,
```
 $ rosrun sis_mobile move_base_client
```
After you run the node, the robot will start to traverse around a 0.6 meter square
#### If you meet any problem during using navigation module, please contact with TA_Sean

## Parameters
* move_base related
  * base_local_planner_params.yaml
    * max_vel_x: maximum forward velocity allowed for the base in meters/sec
    * min_vel_x: minimum forward velocity allowed for the base in meters/sec
    * max_vel_theta: maximum rotational velocity allowed for the base in radians/sec
    * min_vel_theta: minimum rotational velocity allowed for the base in radians/sec
    * min_in_place_vel_theta: minimum rotational velocity allowed for the base while performing in-place rotations in radians/sec
    * acc_lim_theta: rotational acceleration limit of the robot in radians/sec^2
    * acc_lim_x: x acceleration limit of the robot in meters/sec^2
    * acc_lim_y: y acceleration limit of the robot in meters/sec^2
    * xy_goal_tolerance: tolerance in meters for the controller in the x & y distance when achieving a goal
    * yaw_goal_tolerance: tolerance in radians for the controller in yaw/rotation when achieving its goal
    * latch_xy_goal_tolerance: if goal tolerance is latched, if the robot ever reaches the goal xy location it will simply rotate in place, even if it ends up outside the goal tolerance while it is doing so
    * holonomic_robot: determines whether velocity commands are generated for a holonomic or non-holonomic robot. For holonomic robots, strafing velocity commands may be issued to the base. For non-holonomic robots, no strafing velocity commands will be issued
  * costmap_common_params.yaml
    * obstacle_range: maximum range sensor reading that will result in an obstacle being put into the costmap
    * raytrace_range: range to which we will raytrace freespace given a sensor reading
    * footprint: footprint of the robot
    * inflation_radius: set to the maximum distance from obstacles at which a cost should be incurred
  * global_costmap
    * global_frame: global frame for the costmap to operate in
    * robot_base_frame: name of the frame for the base link of the robot
    * update_frequency: frequency in Hz for the map to be updated
    * static_map: whether or not the costmap should initialize itself based on a map served by the map_server. 
    * rolling_window: whether or not to use a rolling window version of the costmap. If the static_map parameter is set to true, this parameter must be set to false.
    * width: width of the map in meters
    * height: height of the map in meters
    * resolution: resolution of the map in meters/cell
  * local_costmap:
    * publish_frequency: frequency in Hz for the map to be publish display information
    ###### Other parameters are same as above one
  * sw_base_local_planner_params.yaml
    * acc_lim_x: x acceleration limit of the robot in meters/sec^2
    * acc_lim_y: y acceleration limit of the robot in meters/sec^2
    * acc_lim_th: rotational acceleration limit of the robot in radians/sec^2
    * max_vel_x: maximum x velocity for the robot in m/s
    * min_vel_x: minimum x velocity for the robot in m/s, negative for backwards motion
    * max_vel_y: maximum y velocity for the robot in m/s
    * min_vel_y: minimum y velocity for the robot in m/s
    * max_trans_vel: absolute value of the maximum translational velocity for the robot in m/s
    * min_trans_vel: absolute value of the minimum translational velocity for the robot in m/s
    * max_rot_vel: absolute value of the maximum rotational velocity for the robot in rad/s 
    * min_rot_vel: absolute value of the minimum rotational velocity for the robot in rad/s
    * yaw_goal_tolerance: tolerance in radians for the controller in yaw/rotation when achieving its goal 
    * xy_goal_tolerance: tolerance in meters for the controller in the x & y distance when achieving a goal
    * latch_xy_goal_tolerance: if goal tolerance is latched, if the robot ever reaches the goal xy location it will simply rotate in place, even if it ends up outside the goal tolerance while it is doing so
* map related
  * sis_competition_map.yaml
    * image: path to the image file containing the occupancy data
    * resolution: resolution of the map, meters / pixel
    * origin: the 2-D pose of the lower-left pixel in the map, as (x, y, yaw)
    * occupied_thresh: pixels with occupancy probability greater than this threshold are considered completely occupied
    * free_thresh: pixels with occupancy probability less than this threshold are considered completely free
    * negate: whether the white/black free/occupied semantics should be reversed
* localization related
  * robot_ekf_config.yaml
    * frequency:
    * two_d_mode:
    * publish_tf:
    * map_frame:
    * odom_frame:
    * base_link_frame:
    * world_frame:
    * odom0:
    * odom0_config:
    * odom0_differential:
    * odom0_relative:
    * odom0_pose_rejection_threshold:
    * odom0_twist_rejection_threshold:
    * pose0:
    * use_control
###### This file is similiar with base_local_planner_params.yaml while this is for mecanum mobile robot
## Robot spec
* Motors: 12V, 36RPM  
#TODO: robot engineering drawing

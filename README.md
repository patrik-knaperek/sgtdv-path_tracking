# **path_tracking package**

___

© **SGT Driverless**

**Authors:** Tereza Ábelová, Juraj Krasňanský, Patrik Knaperek

**Objective:** Computing control commands for speed (throttle/brake) and steering angle based on planned trajectory and vehicle state.

___

## Overview

The path-tracking is divided into two tasks: steering control and speed control.

1. For steering control, [Pure Pursuit](https://drive.google.com/file/d/1ObsUo9i07dW73RavOTAYJBq5Mh6H2AWu/view?usp=share_link) algorithm is implemented. The look-ahead distance is linearly dependent from the current speed, scaled by `k` parameter.

2. For speed control, a simple discrete PI regulator (with ramp) is implemented. Constant reference speed can be set by a service call.

Tested with FSSIM and RC car.

### ROS Interface

**Subscribed topics**
* `/path_planning/trajectory` [[`sgtdv_msgs/Point2DArr`](../sgtdv_msgs/msg/Point2DArr.msg)] : reference trajectory array in the `map` frame
* `/odometry/pose` [[`sgtdv_msgs/CarPose`](../sgtdv_msgs/msg/CarPose.msg)] : current pose in the global coordinate system
* `/odometry/velocity` [[`sgtdv_msgs/CarVel`](../sgtdv_msgs/msg/CarVel.msg)] : current velocity in the `base_link` frame

**Published topics**
* `/path_tracking/cmd` [[`sgtdv_msgs/Control`](../sgtdv_msgs/msg/Control.msg)] : speed and steering control command

*If `SGT_VISUALIZE` macro enabled*
* `/path_tracking/visualize/pure_pursuit` [[`visualization_msgs/Marker`](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html)] : visualization significant points for PP algorithm: target (lookahead) point, the closest trajectory point to the rear axle, rear axle position, front axle position
* `/path_tracking/visualize/steering` [[`geometry_msgs/PoseStamped`](https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseStamped.html)] : visualization of the desired steering pose (rotation)

*If `SGT_DEBUG_STATE` macro enabled*
* `/path_tracking/debug_state` [[`sgtdv_msgs/DebugState`](../sgtdv_msgs/msg/DebugState.msg)] : node lifecycle information (active/inactive, current speed and steering command)

**Advertised services**
* `/path_tracking/start` [[`std_srvs/Empty`](http://docs.ros.org/en/noetic/api/std_srvs/html/srv/Empty.html)] : "start moving" command → speed command given by the speed controller
* `/path_tracking/stop` [[`std_srvs/Empty`](http://docs.ros.org/en/noetic/api/std_srvs/html/srv/Empty.html)] : "stop moving" command → speed command equals zero
* `/path_tracking/set_speed` [[`sgtdv_msgs/Float32Srv`](../sgtdv_msgs/srv/Float32Srv.srv)] : set the reference speed

**Parameters**
* resulting from the setup:
    - `/vehicle/car_length` : distance [m] between axles
    - `/vehicle/rear_wheels_offset` : distance [m] from COG (center of vehicle frame) to the rear axle
    - `/vehicle/front_wheels_offset` : distance [m] from COG (center of vehicle frame) to the front axle
    - `/controller/speed/min`, `controller/speed/max` : [m/s] range of speed control output
    - `/controller/steering/min`, `controller/steering/max` : [rad] range of steering control output
    - `/loop_track` : after reaching the last point of the trajectory, wether to stop or navigate towards the first point again
* may be used for controller tunning:
    - `/controller/speed/p` : P gain of speed controller
    - `/controller/speed/i` : I gain of speed controller
    <!-- - `controller/speed/ref_speed` : constant reference speed -->
    - `/controller/speed/speed_raise_rate` : maximum frequency of speed control output increment
    - `/controller/steering/k` : ref. to the equation in the [Section 2.2.1](https://drive.google.com/file/d/1ObsUo9i07dW73RavOTAYJBq5Mh6H2AWu/view?usp=share_link)
    - `/controller/steering/lookahead_dist_min`,  `/controller/steering/lookahead_dist_max`: range of look-ahead distance, ref. to [Figure 10](https://drive.google.com/file/d/1ObsUo9i07dW73RavOTAYJBq5Mh6H2AWu/view?usp=share_link)

### Related packages
* [`path_planning`](../path_planning/README.md) : `/path_planning/trajectory` publisher and `/path_tracking/set_speed` caller
* [`ptp_trajectory`](../ptp_trajectory/README.md) : (alternating `path_planning`) `/path_planning/trajectory` publisher and `path_tracking/start`, `/path_tracking/stop` caller
* [`odometry_interface`](../odometry_interface/README.md) : `/odometry/velocity` and `/odometry/pose` publisher
* [`control_si`](../simulation_interface/control_si/README.md) : (FSSIM setup) : `/odometry/pose`, `/odometry/velocity` publisher and `/path_tracking/cmd` subscriber
* [`vesc_interface`](../racecar_interface/vesc_interface/README.md) : (RC car setup) `/path_tracking/cmd` subscriber
* [`jetson_can_interface`](../jetson_can_interface/README.md) : (formula setup) `/path_tracking/cmd` subscriber (*not properly tested yet*)

## Compilation
* standalone
```sh
$ cd ${SGT_ROOT}
$ catkin build path_tracking
```
* RC car setup
```sh
$ source ${SGT_ROOT}/scripts/build_rc.sh
```
* FSSIM setup
```sh
$ source ${SGT_ROOT}/scripts/build_sim.sh
```

### Compilation configuration
* [`SGT_Macros.h`](../SGT_Macros.h)
	- `SGT_VISUALIZATION` : publish intermediate calculations on visualizable topics
        - `/path_tracking/visualize/target [visualization_msgs/Marker]` - lookahead (target) point, closest trajectory point, rear wheelbase position, front wheelbase position
        - `/path_tracking/visualize/steering [geometry_msgs/PoseStamped]` - steering angle command
    - `SGT_DEBUG_STATE` : publish node lifecycle information
    

## Launch
* standalone
```sh
$ source ${SGT_ROOT}/ros_implementation/devel/setup.bash
$ roslaunch path_tracking path_tracking.launch
```
* RC car setup
```sh
$ source ${SGT_ROOT}/ros_implementation/devel/setup.bash
$ roslaunch master rc.launch
```
* FSSIM setup (check [FSSIM testing](../../doc/FSSIM_testing.md) manual for more info)
```sh
$ source ${SGT_ROOT}/ros_implementation/devel/setup.bash
$ roslaunch master trackdrive.launch
```

To start driving, the trajectory, pose and velocity topics must be active. In a new terminal run
```sh
$ source ${SGT_ROOT}/ros_implementation/devel/setup.bash
$ rosservice call /path_tracking/set_speed "data: X.Y"
$ rosservice call /path_tracking/start "{}"
```
### Launch configuration
* [`path_tracking.yaml`](../path_tracking/params/path_tracking.yaml) : default setup 
* [`path_tracking_rc.yaml`](../path_tracking/params/path_tracking_rc.yaml) : RC car setup
* [`path_tracking_sim.yaml`](../path_tracking/params/path_tracking_sim.yaml) : FSSIM setup

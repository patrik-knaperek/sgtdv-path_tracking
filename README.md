# **PathTracking package**

___

© **SGT Driverless**

**Authors:** Tereza Ábelová, Juraj Krasňanský, Patrik Knaperek

**Objective:** Computing control commands for speed (throttle/brake) and steering angle based on planned trajectory and vehicle state.

___

The path-tracking task is divided into two: steering control and speed control.

For steering control, [Pure Pursuit](https://drive.google.com/file/d/1ObsUo9i07dW73RavOTAYJBq5Mh6H2AWu/view?usp=share_link) algorithm is implemented. For speed control, a simple discrete PI regulator (with ramp) is implemented. Constant reference speed is given as a parameter.

Tested with FSSIM and RC car.

### Related packages
* [`path_planning`](../path_planning/README.md)
* [`ptp_trajectory`](../ptp_trajectory/README.md)
* [`control_si`](../simulation_interface/control_si/README.md)
* `jetson_can_interface`
* `racecar-interface`

## ROS Interface

### Subscribed topics
* `path_planning/trajectory`[`[sgtdv_msgs/Point2DArr]`](../sgtdv_msgs/msg/Point2DArr.msg) - reference trajectory array
* `odometry/pose`[`[sgtdv_msgs/CarPose]`](../sgtdv_msgs/msg/CarPose.msg) - current pose in the global coordinate system
* `odometry/velocity`[`[sgtdv_msgs/CarVel]`](../sgtdv_msgs/msg/CarVel.msg) - current velocity in the base coordinate system

### Published topics
* `path_tracking/cmd`[`[sgtdv_msgs/Control]`](../sgtdv_msgs/msg/Control.msg) - speed and steering control command
Optional
* `path_tracking/visualize/pure_pursuit`[`[visualization_msgs/Marker]`](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html) - visualization significant points for PP algorithm: target (lookahead) point, the closest trajectory point to the rear axle, rear axle position, front axle position
* `path_tracking/visualize/steering`[`[geometry_msgs/PoseStamped]`](https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseStamped.html) - visualization of the desired steering pose (rotation)
* `path_tracking/debug_state`[`[sgtdv_msgs/DebugState]`](../sgtdv_msgs/msg/DebugState.msg) - debug visualization data

### Advertised services
* `path_tracking/start`[`[std_srvs/Empty]`](http://docs.ros.org/en/noetic/api/std_srvs/html/srv/Empty.html) - "start moving" command → speed command given by the speed controller
* `path_tracking/stop`[`[std_srvs/Empty]`](http://docs.ros.org/en/noetic/api/std_srvs/html/srv/Empty.html) - "stop moving" command → speed command equals zero
* `path_tracking/set_speed`[`[sgtdv_msgs/Float32Srv]`](../sgtdv_msgs/srv/Float32Srv.srv) - set the reference speed

## Compilation
```sh
$ cd ${SGT_ROOT}/ros_implementation
$ catkin build path_tracking
```

### Compilation configuration
* [`SGT_Macros.h`](../SGT_Macros.h)
	* `SGT_VISUALIZATION` : publish intermediate calculations on visualizable topics
        - `/path_tracking/visualize/target [visualization_msgs/Marker]` - lookahead (target) point, closest trajectory point, rear wheelbase position, front wheelbase position
        - `/path_tracking/visualize/steering [geometry_msgs/PoseStamped]` - steering angle command

## Launch
```sh
$ source ${SGT_ROOT}/ros_implementation/devel/setup.bash
$ roslaunch path_tracking path_tracking.launch
```
To start driving, the trajectory, pose and velocity topics must be active. In a new terminal run
```sh
$ source ${SGT_ROOT}/ros_implementation/devel/setup.bash
$ rosservice call /path_tracking/set_speed "data: X.Y"
$ rosservice call /path_tracking/start "{}"
```

### Launch with FSSIM 
([Requires AMZ FSD skeleton & FSSIM installed and launched](https://gitlab.com/sgt-driverless/simulation/fsd_skeleton/-/blob/sgt-noetic-devel/SGT-DV_install_man.md))
```sh
$ roslaunch control_si control_si
```
### Launch on RC car
```sh
$ roslaunch path_tracking path_tracking_rc.launch
```

In a new terminal:
```sh
$ source <path_to_racecar-interface_pkg>/devel/setup.bash
$ ./start.bash
```

### Launch configuration
* `path_tracking.yaml`, `path_tracking_rc.yaml`, `path_tracking_sim.yaml`
* resulting from the setup:
    - `car_length` : distance [m] between axles
    - `rear_wheels_offset` : distance [m] from COG (center of vehicle frame) to the rear axle
    - `front_wheels_offset` : distance [m] from COG (center of vehicle frame) to the front axle
    - `controller/speed/min`, `controller/speed/max` : range of speed control output
    - `controller/steering/min`, `controller/steering/max` : range of steering control output
* may be used for tunning:
    - `controller/speed/p` : P gain of speed controller
    - `controller/speed/i` : I gain of speed controller
    <!-- - `controller/speed/ref_speed` : constant reference speed -->
    - `controller/speed/speed_raise_rate` : maximum frequency of speed control output increment
    - `controller/steering/k` : ref. to the equation in the [Section 2.2.1](https://drive.google.com/file/d/1ObsUo9i07dW73RavOTAYJBq5Mh6H2AWu/view?usp=share_link)
    - `controller/steering/lookahead_dist_min`,  `controller/steering/lookahead_dist_max`: range of look-ahead distance, ref. to [Figure 10](https://drive.google.com/file/d/1ObsUo9i07dW73RavOTAYJBq5Mh6H2AWu/view?usp=share_link)
# **PathTracking package**

___

© **SGT Driverless**

**Authors:** Tereza Ábelová, Juraj Krasňanský, Patrik Knaperek

**Objective:** Computing control values for motors and steering actuator based on planned trajectory and vehicle state.

___

[Pure Pursuit](https://drive.google.com/file/d/1ObsUo9i07dW73RavOTAYJBq5Mh6H2AWu/view?usp=share_link) steering control algorithm is implemented. Stanley algorithm implementation is not working. For speed control, a simple discrete PI regulator with ramp is implemented. Constant reference speed is given as a parameter. Tested with FSSIM and RC car.

### Related packages
* `path_planning`
* `ptp_trajectory`
* `path_tracking_sim_interface`
* `racecar-interface`
* `jetson_can_interface`

## Compilation
```sh
$ cd <path_to_SGT_workspace>/ros_implementation
$ catkin build path_tracking
```

## Launch
```sh
$ source <path_to_SGT_workspace>/ros_implementation/devel/setup.bash
$ roslaunch path_tracking path_tracking.launch
```
### Launch with FSSIM 
([Requires AMZ FSD skeleton & FSSIM installed](https://gitlab.com/sgt-driverless/simulation/fsd_skeleton/-/blob/sgt-noetic-devel/SGT-DV_install_man.md))
```sh
$ roslaunch path_tracking_sim_interface path_tracking_sim_interface
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
    - `controller/speed/ref_speed` : constant reference speed
    - `controller/speed/speed_raise_rate` : maximum frequency of speed control output increment
    - `controller/steering/k` : ref. to the equation in the [Section 2.2.1](https://drive.google.com/file/d/1ObsUo9i07dW73RavOTAYJBq5Mh6H2AWu/view?usp=share_link)
    - `controller/steering/lookahead_dist_min`,  `controller/steering/lookahead_dist_max`: range of look-ahead distance, ref. to [Figure 10](https://drive.google.com/file/d/1ObsUo9i07dW73RavOTAYJBq5Mh6H2AWu/view?usp=share_link)
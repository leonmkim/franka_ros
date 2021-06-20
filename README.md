# ROS integration for Franka Emika research robots

[![Build Status][travis-status]][travis]

See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.

## Changes in this (feature_rosparam_joint_limits) branch of franka_ros
The Franka ROS interfaces utilizes the [ros_control](http://wiki.ros.org/ros_control) framework and specifically the [joint_limits_interface](http://docs.ros.org/en/melodic/api/joint_limits_interface/html/c++/index.html) to enforce limits on joint positions/velocities/accelerations/jerks by saturating user commands. The values for these bounds can be loaded from two sources, with the most recent loaded setting overwriting the other source. However, out of the box, franka_ros only seems to support loading from the loaded URDF of the robot. The changes in this branch additionally allow loading from the ros parameter server which will overwrite the specified bounds from the URDF. These parameters can be loaded from a yaml file like [this one](https://github.com/leonmkim/franka_ros_interface/blob/40a92d82777295ff0e1c722bba6601aedde8e69a/franka_interface/config/robot_config.yaml#L5).        

## License

All packages of `franka_ros` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs
[travis-status]: https://travis-ci.org/frankaemika/franka_ros.svg?branch=kinetic-devel
[travis]: https://travis-ci.org/frankaemika/franka_ros

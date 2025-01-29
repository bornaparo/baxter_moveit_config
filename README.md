## Package for using moveit2 and gazebo ignition with baxter robot in ROS2

### Requires baxter_description from [this fork](https://github.com/bornaparo/baxter_common_ros2.git)

### Usage:
#### Using it only in rviz:
`ros2 launch baxter_moveit_config baxter_moveit_opaque.launch.py`

#### Using it with gazebo ignition:
`ros2 launch baxter_moveit_config baxter_gazebo_moveit.launch.py`

### Credits: 
Most of the configuration files were taken from [baxter_moveit_config](https://github.com/arne48/baxter_moveit_config/tree/master) package for ROS1
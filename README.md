# irsl_ros_control_shm

ROS2 control hardware interface backed by the shared-memory transport from irsl_shm_controller_library.

## Build

- This package depends on libraries from https://github.com/IRSL-tut/irsl_shm_controller_library.
- Optional Dynamixel support is enabled when `dynamixel_hardware_shm` is available at configure time.

## Usage

This package now provides a ROS2 control `hardware_interface::SystemInterface` plugin instead of a ROS1 standalone node.

Use the plugin name `irsl_ros_control_shm/RobotHWShm` inside a `<ros2_control>` block in your robot description and run it with `controller_manager`'s `ros2_control_node`.

The sample files under `test/` show the expected ROS2 controller YAML and a minimal URDF block. A ready-to-run example launch file is installed under `launch/ros2_control_shm.launch.py`.

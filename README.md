# SOLO 12
## Running SOLO 12
### Installations
In addition to ROS2, you will need (assuming you are using Galactic):
- `sudo apt install ros-galactic-xacro`
- `sudo apt install ros-galactic-gazebo-ros`

### Usage
In your workspace folder:
1. `colcon build` (if you have not built it yet)
2. `source /opt/ros/galactic/setup.bash`
3. `source install/local_setup.bash`
4. `ros2 launch ros2_solo_12 view_solo.launch.py`
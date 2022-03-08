# SOLO 12
## Running SOLO 12
### Installations
In addition to ROS2, you will need (assuming you are using Galactic):
- `sudo apt install ros-galactic-xacro`
- `sudo apt install ros-galactic-gazebo-ros`

### Usage
First, add the package "ros2\_solo\_12" in the src folder of your workspace. Then, in your workspace folder:
1. `rosdep install --from-paths src --ignore-src -r -y` (to install dependencies)
2. `colcon build` (if you have not built it yet)
3. `source /opt/ros/galactic/setup.bash`
4. `source install/local_setup.bash`
5. `ros2 launch ros2_solo_12 view_solo.launch.py`

# Walker robot controller using state machines

This repo contains a ROS2 package for autonomous control of a turtlebot waffle robot. 
The robot moves straight until it detects an obstacle at a distance of 1 meter infront of it.
Then the robot rotates alcockwise until the obstacle is clear and moves straight. It repeats this process
and every time it rotates, it changes the direction of rotation.

It moves straight with a speed of 0.3 m/s and rotates with a speed of 1.0 rad/s.

The robot is always in either of the two states namely Forward or Rotation.

Prerequisites:

1. ROS2 Humble
2. Turtlebot3 packages
3. Gazebo Simulator


### Installation

```bash
# Install ROS2 Turtlebot3 packages if not already installed
sudo apt update
sudo apt install ros-humble-turtlebot3*

# Create workspace
mkdir -p ~/ros2_ws
cd ~/ros2_ws

# Clone the repository
mkdir src
cd src
git clone https://github.com/piyush-g0enka/my_gazebo_tutorials.git

# Install dependencies
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src -y -i

# Build the package
colcon build

# Source the workspace
source install/setup.bash
```

### Setup

```bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models/
```

### Running Simulation

```bash
# Launch Gazebo world with wrobot
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Launch controller node
$ ros2 launch walker node.launch.py
```

### Bagfiles

```bash
# Launch controller node with rosbag recording
$ ros2 launch walker node.launch.py record:=true

# Playback the bag file 
$ ros2 bag play path/to/your/bag
```

### Cpplint

```bash
# Run below command from package directory
$ cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/my_walker.cpp src/main.cpp
```

### Clang-tidy

```bash
# Create symbolic link to compile_commands.json in package directory
$ ln -s ~/<path-to-ros2_ws>/build/my_gazebo_tutorials/compile_commands.json ~/<path-to-ros2_ws>/src/my_gazebo_tutorials

# Get clang-tidy output - Run from package directory
$ clang-tidy -p compile_commands.json --extra-arg=-std=c++17 src/my_walker.cpp src/main.cpp ; echo "Exit code: $?"
```

# METR4202 — Autonomous Exploration and Target Search

This project uses a TurtleBot3 Waffle Pi to explore an unknown environment.
It builds a map using SLAM, detects exploration frontiers, plans an order in
which to visit them, and sends navigation goals to Nav2.

## Running the exploration demo

### Prerequisites

The demo requires the course's ROS 2 Humble, TurtleBot3 simulation, Gazebo,
SLAM Toolbox, Nav2 and RViz2 setup.

The project also uses NetworkX, NumPy and SciPy. Install these additional
Python packages and Nav2 Simple Commander if they are not already installed:

```bash
sudo apt update
sudo apt install python3-networkx python3-numpy python3-scipy \
  ros-humble-nav2-simple-commander
```

### Download and build

Clone this repository into a ROS 2 workspace. 

```bash
mkdir -p ~/metr4202_demo_ws/src
cd ~/metr4202_demo_ws/src
git clone https://github.com/Reuben678/metr4202_2026_team30.git

cd ~/metr4202_demo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Launch the demo

```bash
ros2 launch project_explore autonomous_exploration.launch.py
```

The launch file starts the TurtleBot3 Gazebo world, SLAM, Nav2, RViz2, the
frontier search and utility nodes, and the Exploration Manager. Gazebo and Rviz2 take some time 
to start up.

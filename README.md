# Obstacle Avoidance Simulation

ROS/Gazebo obstacle-avoidance and right-side wall-following controllers for the Triton robot. The repository contains two catkin packages for COMPSCI 603 Project 2:

- `P2D1`: a manually designed Q-table wall-following controller.
- `P2D2`: reinforcement-learning wall following with Q-learning and SARSA training/testing modes.

Both packages are intended to run in a ROS Noetic catkin workspace alongside the `stingray_sim` package, which provides the Triton robot model, Gazebo worlds, and simulator launch files.

## Repository layout

```text
.
├── P2D1/
│   ├── launch/p2d1.launch          # Gazebo + manual Q-table controller launch file
│   ├── scripts/
│   │   ├── q_table.py              # Hand-authored Deliverable 1 Q-table
│   │   ├── set_triton_pose.py      # Utility to set the Triton start pose
│   │   └── wall_follow_node.py     # Manual wall-following ROS node
│   ├── CMakeLists.txt
│   └── package.xml
└── P2D2/
    ├── data/                       # Learned Q-tables and reward logs
    ├── launch/p2d2.launch          # Gazebo + RL controller launch file
    ├── scripts/
    │   ├── q_table.py              # Initial policy seed for RL
    │   ├── rl_wall_follow_node.py  # Q-learning/SARSA ROS node
    │   └── set_triton_pose.py      # Utility to set the Triton start pose
    ├── CMakeLists.txt
    └── package.xml
```

## Prerequisites

- Ubuntu with ROS Noetic and Gazebo support.
- A catkin workspace, for example `~/catkin_ws`.
- The `stingray_sim` package checked out into the same workspace as this repository.
- ROS package dependencies used by the nodes:
  - `rospy`
  - `geometry_msgs`
  - `sensor_msgs`
  - `std_msgs`
  - `gazebo_msgs`

Expected workspace layout:

```text
~/catkin_ws/src/
├── obstacle_avoidance_sim/
│   ├── P2D1/
│   └── P2D2/
└── stingray_sim/
```

## Setup

From a catkin workspace containing this repository and `stingray_sim`:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

If ROS cannot find the packages, confirm that `P2D1`, `P2D2`, and `stingray_sim` are all under the workspace `src/` directory and rebuild the workspace.

## Running Deliverable 1: manual Q-table controller

Launch Gazebo with the Triton robot and the manual wall-following node:

```bash
roslaunch P2D1 p2d1.launch
```

The `P2D1` controller:

- Subscribes to `/scan` for LiDAR data.
- Publishes velocity commands on `/cmd_vel`.
- Tracks a right-side wall using front-right and front obstacle state features.
- Uses a manually defined Q-table policy with fixed motion primitives such as fast forward, slow forward, soft turns, and hard turns.

Important launch parameters are defined in `P2D1/launch/p2d1.launch`, including desired wall distance (`d_w`), front obstacle threshold (`front_blocked`), motion primitive speeds, and control rate.

## Running Deliverable 2: reinforcement learning controller

`P2D2` supports Q-learning and SARSA. The controller can either train a policy or test a saved policy.

### Train Q-learning

```bash
roslaunch P2D2 p2d2.launch algorithm:=q_learning mode:=train
```

### Test Q-learning

```bash
roslaunch P2D2 p2d2.launch algorithm:=q_learning mode:=test
```

### Train SARSA

```bash
roslaunch P2D2 p2d2.launch algorithm:=sarsa mode:=train
```

### Test SARSA

```bash
roslaunch P2D2 p2d2.launch algorithm:=sarsa mode:=test
```

The `P2D2` controller:

- Uses a discrete state made from front-right distance, front obstacle status, and front-left distance.
- Uses three actions: `FWD`, `LEFT`, and `RIGHT`.
- Seeds learning from a simplified version of the Deliverable 1 manual policy.
- Saves learned Q-tables, best Q-table checkpoints, and per-episode reward logs in `P2D2/data/`.

The default training configuration in `P2D2/launch/p2d2.launch` uses 300 episodes with 300 steps per episode. You can override launch arguments and parameters from the command line as needed.

## Data artifacts

The `P2D2/data/` directory contains saved training outputs:

- `q_learning_qtable.pkl`
- `q_learning_best_qtable.pkl`
- `q_learning_rewards.csv`
- `sarsa_qtable.pkl`
- `sarsa_best_qtable.pkl`
- `sarsa_rewards.csv`

Testing mode loads the corresponding `*_best_qtable.pkl` file for the selected algorithm.

## Useful troubleshooting notes

- Always run `source devel/setup.bash` in each new terminal before launching ROS nodes.
- Make sure Gazebo services such as `/gazebo/set_model_state`, `/gazebo/get_model_state`, and `/gazebo/unpause_physics` are available after launch.
- If the robot spawns in an unfavorable position during testing, stop and restart the launch command or use the pose-setting utility/service to reposition it.
- If package discovery fails, rebuild with `catkin_make` from the workspace root and verify the workspace layout.

## License

The ROS package manifests declare the project license as BSD.

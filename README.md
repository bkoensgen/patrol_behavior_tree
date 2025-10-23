# Patrol Behavior Tree

Autonomous robot patrol implementation using BehaviorTree.CPP and ROS 2.

## Features

- Reactive obstacle avoidance
- Behavior tree architecture
- Gazebo simulation support
- Unit tests with gtest

## Dependencies

- ROS 2 Humble
- BehaviorTree.CPP 4.6
- Gazebo (optional, for simulation)

## Build
```bash
cd ~/ros2_ws
colcon build --packages-select patrol_behavior_tree
source install/setup.bash
```

## Run
```bash
ros2 run patrol_behavior_tree patrol_bt_node
```

With simulation:
```bash
ros2 launch patrol_behavior_tree patrol_demo.launch.py
```

## Test
```bash
colcon test --packages-select patrol_behavior_tree
colcon test-result --verbose
```

## Behavior

The robot patrols forward continuously. When an obstacle is detected within 0.5m:
1. Rotate 90 degrees
2. Move forward 0.5m
3. Resume normal patrol

## Architecture
```
ReactiveSequence
├── Fallback (obstacle handling)
│   ├── Inverter(IsObstacleClose)
│   └── Sequence (avoidance)
│       ├── RotateRobot
│       └── MoveForward
└── MoveForward (patrol)
```
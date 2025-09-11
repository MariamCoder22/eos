#!/bin/bash
# Run simulation for Eos OS

# Start Gazebo with TurtleBot3 world
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
sleep 5

# Start the ROS bridge for SNN (Python side)
python3 python/ros_bridge.py &

# Start the Eos OS node (Rust side)
cargo run

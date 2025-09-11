#!/bin/bash
# Setup script for Eos OS

# -----------------------------
# Install Rust
# -----------------------------
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# -----------------------------
# Install ROS 2 (assuming Ubuntu)
# -----------------------------
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release

# Add ROS 2 key and repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Foxy Desktop
sudo apt update && sudo apt install -y ros-foxy-desktop

# -----------------------------
# Install ROS 2 Rust bindings dependencies
# -----------------------------
sudo apt install -y libclang-dev

# -----------------------------
# Install Python dependencies
# -----------------------------
pip3 install numpy

# -----------------------------
# Build the Rust project
# -----------------------------
cargo build

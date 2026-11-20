<div align="right">

[English](README.md)|[简体中文](README_CN.md)

</div>

# Revo2 Description

## Overview

The Revo2 Description repository offers detailed URDF models for BrainCo Revo2 dexterous hands. Additionally, the repository provides Docker support for easy visualization and simulation setup.

## Features

- **Detailed 3D Models**: High-fidelity 3D models of Revo2 left and right hands for accurate simulation and visualization.
- **RViz Visualization**: Built-in launch files for visualizing robot models in RViz.
- **Docker Support**: Pre-configured Docker environment for easy setup and deployment.

## Prerequisites

### For Docker Usage
- Docker

### For Native ROS2 Usage
- Ubuntu 20.04/22.04
- ROS 2 (Humble/Iron/Jazzy/Rolling)
- Additional packages: `ros-<distro>-xacro`, `ros-<distro>-joint-state-publisher-gui`

## Installation and Setup

### Option 1: Docker Setup (Recommended)

To use the Docker-based visualization:

#### Automatic Setup (Recommended)
The scripts will automatically build the Docker image on first use:

```bash
# Visualize left hand (default)
./scripts/visualize_revo2.sh left

# Visualize right hand
./scripts/visualize_revo2.sh right
```

#### Manual Setup (Optional)
You can also build the Docker image manually beforehand:

```bash
# Build the Docker image once
docker build -t revo2_description .docker

# Then use the scripts (they will skip building and use the existing image)
./scripts/visualize_revo2.sh left
```

### Option 2: Native ROS2 Setup

For native ROS2 installation:

```bash
# Install ROS 2 (if not already installed)
# Follow instructions at: https://docs.ros.org/en/humble/Installation.html

# Install additional dependencies
sudo apt-get update
sudo apt-get install ros-<distro>-xacro ros-<distro>-joint-state-publisher-gui

# Create a ROS 2 workspace
mkdir -p ~/revo2_ws/src
cd ~/revo2_ws/src

# Copy or clone the revo2_description package
cp -r /path/to/revo2_description .

# Build the workspace
cd ~/revo2_ws
colcon build --packages-select revo2_description --symlink-install
source install/setup.bash
```

## Visualization

### RViz Visualization

#### Using Docker (Recommended)

```bash
# Visualize left hand (default)
./scripts/visualize_revo2.sh left

# Visualize right hand
./scripts/visualize_revo2.sh right
```

#### Using Native ROS2

After setting up the workspace:

```bash
# Source the workspace
source ~/revo2_ws/install/setup.bash

# Visualize left hand
ros2 launch revo2_description view_revo2_left_hand.launch.py

# Visualize right hand
ros2 launch revo2_description view_revo2_right_hand.launch.py
```

## Joint Information

### Left Hand Joints

| Joint Name | Description | Range (degrees) | Range (radians) |
|------------|-------------|-----------------|-----------------|
| left_thumb_flex_joint | Thumb flexion | 0 ~ 59 | 0 ~ 1.03 |
| left_thumb_abduct_joint | Thumb abduction | 0 ~ 90 | 0 ~ 1.57 |
| left_index_joint | Index finger | 0 ~ 81 | 0 ~ 1.41 |
| left_middle_joint | Middle finger | 0 ~ 81 | 0 ~ 1.41 |
| left_ring_joint | Ring finger | 0 ~ 81 | 0 ~ 1.41 |
| left_pinky_joint | Pinky finger | 0 ~ 81 | 0 ~ 1.41 |

### Right Hand Joints

| Joint Name | Description | Range (degrees) | Range (radians) |
|------------|-------------|-----------------|-----------------|
| right_thumb_flex_joint | Thumb flexion | 0 ~ 59 | 0 ~ 1.03 |
| right_thumb_abduct_joint | Thumb abduction | 0 ~ 90 | 0 ~ 1.57 |
| right_index_joint | Index finger | 0 ~ 81 | 0 ~ 1.41 |
| right_middle_joint | Middle finger | 0 ~ 81 | 0 ~ 1.41 |
| right_ring_joint | Ring finger | 0 ~ 81 | 0 ~ 1.41 |
| right_pinky_joint | Pinky finger | 0 ~ 81 | 0 ~ 1.41 |

## Package Structure

```
revo2_description/
├── launch/                 # ROS launch files
├── meshes/                 # 3D mesh files (.STL)
│   ├── revo2_left_hand/    # Left hand meshes
│   └── revo2_right_hand/   # Right hand meshes
├── scripts/                # Docker utility scripts
├── urdf/                   # URDF model files
├── rviz/                   # RViz configuration files
├── .docker/               # Docker support files
├── CMakeLists.txt
├── package.xml
├── CHANGELOG.rst
└── README.md
```

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

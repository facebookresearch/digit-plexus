
# Digit Plexus 
This guide will help you set up the environment, build the ROS2 package Digit Plexus , and join the growing community of developers and researchers.

## 🛠 Virtual Environment Setup

To install and activate the virtual environment for Digit Plexus, run the following commands:

```
cd digit-plexus
./plexus/install_plexus_venv.sh
```
This will handle the necessary dependencies and ensure your environment is ready for development.

## 🛠 Build ROS2 pkg
```
mkdir -p plexus_ws/src
cd plexus_ws/src
git clone https://github.com/facebookresearch/digit-plexus.git
cd ..
sh ./src/digit-plexus/plexus/ros2/plexus_pkg/colcon_build.sh
```

## 🛠 Install Digit Plexus  pkg
```
cd plexus_ws/src/digit-plexus/
pip install -e .
```

This creates the workspace, builds the source, and prepares everything for use with ROS2 nodes and topics.

## 🚀 Run Commands 
To launch the Digit Plexus package, use the following command:

This launches the allegro hand with plexus.
```bash
ros2 launch plexus_pkg allegro_hand_launch.py
```

For Sensor Patch
```
ros2 launch plexus_pkg sensor_patch_launch.py
```

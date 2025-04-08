#!/bin/bash
# filepath: launch.sh

# Exit on error
set -e

# Define colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== ROS 2 Humble + Gazebo Fortress STL Model Launcher ===${NC}\n"

# Function to check if a package is installed
check_installed() {
  if command -v $1 >/dev/null 2>&1; then
    return 0
  else
    return 1
  fi
}

# Check if running as root and exit if so
if [ "$EUID" -eq 0 ]; then
  echo -e "${RED}Please do not run this script as root or with sudo${NC}"
  exit 1
fi

# Check and install required system packages
echo -e "${YELLOW}Checking and installing required system packages...${NC}"
SYSTEM_DEPS="wget curl gnupg2 lsb-release build-essential cmake git python3-pip"

for pkg in $SYSTEM_DEPS; do
  if ! dpkg -l | grep -q "ii  $pkg"; then
    echo -e "${YELLOW}Installing $pkg...${NC}"
    sudo apt-get update && sudo apt-get install -y $pkg
  fi
done

# Check if ROS 2 Humble is installed, if not, install it
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
  echo -e "${YELLOW}ROS 2 Humble not found. Installing...${NC}"
  
  # Add ROS 2 apt repository
  sudo apt-get update && sudo apt-get install -y curl gnupg lsb-release
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  
  # Install ROS 2 Humble
  sudo apt-get update
  sudo apt-get install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions
  
  echo -e "${GREEN}ROS 2 Humble installed successfully${NC}"
else
  echo -e "${GREEN}ROS 2 Humble already installed${NC}"
fi

# Install ROS 2 - Gazebo Fortress integration packages
echo -e "${YELLOW}Installing ROS 2 - Gazebo Fortress integration packages...${NC}"
sudo apt-get install -y ros-humble-ros-gz

# Make sure Gazebo Fortress is installed as part of ros-humble-ros-gz
if ! check_installed gz; then
  echo -e "${YELLOW}Ensuring Gazebo Fortress is properly installed...${NC}"
  sudo apt-get install -y gz-fortress
  echo -e "${GREEN}Gazebo Fortress installed successfully${NC}"
else
  echo -e "${GREEN}Gazebo Fortress already installed${NC}"
fi

echo -e "${GREEN}Starting ROS 2 Humble with Gazebo Fortress and loading STL model...${NC}"

# Check if STL file exists
STL_FILE="clean.stl"
if [ ! -f "$STL_FILE" ]; then
  echo -e "${RED}Error: $STL_FILE not found in the current directory${NC}"
  exit 1
fi

# Create a temporary ROS 2 package for the model
TEMP_WS="$(mktemp -d)/ros2_ws"
mkdir -p $TEMP_WS/src/stl_model/resource
mkdir -p $TEMP_WS/src/stl_model/launch
mkdir -p $TEMP_WS/src/stl_model/worlds
mkdir -p $TEMP_WS/src/stl_model/models/clean_model/meshes

# Copy STL file to models directory
cp "$STL_FILE" $TEMP_WS/src/stl_model/models/clean_model/meshes/

# Create model.config
cat > $TEMP_WS/src/stl_model/models/clean_model/model.config << EOF
<?xml version="1.0"?>
<model>
  <name>clean_model</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
  <name>Gazebo User</name>
  <email>user@gazebo.org</email>
  </author>
  <description>
  Imported STL model
  </description>
</model>
EOF

# Create model.sdf with SDF version compatible with Fortress
cat > $TEMP_WS/src/stl_model/models/clean_model/model.sdf << EOF
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="clean_model">
  <link name="link">
    <inertial>
    <mass>1.0</mass>
    <inertia>
      <ixx>0.083</ixx>
      <ixy>0.0</ixy>
      <ixz>0.0</ixz>
      <iyy>0.083</iyy>
      <iyz>0.0</iyz>
      <izz>0.083</izz>
    </inertia>
    </inertial>
    <collision name="collision">
    <geometry>
      <mesh>
      <uri>model://clean_model/meshes/clean.stl</uri>
      <scale>1 1 1</scale>
      </mesh>
    </geometry>
    </collision>
    <visual name="visual">
    <geometry>
      <mesh>
      <uri>model://clean_model/meshes/clean.stl</uri>
      <scale>1 1 1</scale>
      </mesh>
    </geometry>
    <material>
      <ambient>0.3 0.3 0.3 1</ambient>
      <diffuse>0.7 0.7 0.7 1</diffuse>
      <specular>0.01 0.01 0.01 1</specular>
    </material>
    </visual>
  </link>
  <static>true</static>
  </model>
</sdf>
EOF

# Create world file for Fortress
cat > $TEMP_WS/src/stl_model/worlds/clean_world.sdf << EOF
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="clean_world">
  <physics name="1ms" type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>
  <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"></plugin>
  <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors"></plugin>
  <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"></plugin>
  <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"></plugin>
  
  <light type="directional" name="sun">
    <cast_shadows>true</cast_shadows>
    <pose>0 0 10 0 0 0</pose>
    <diffuse>0.8 0.8 0.8 1</diffuse>
    <specular>0.2 0.2 0.2 1</specular>
    <direction>-0.5 0.1 -0.9</direction>
  </light>
  
  <model name="ground_plane">
    <static>true</static>
    <link name="link">
    <collision name="collision">
      <geometry>
      <plane>
        <normal>0 0 1</normal>
        <size>100 100</size>
      </plane>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
      <plane>
        <normal>0 0 1</normal>
        <size>100 100</size>
      </plane>
      </geometry>
      <material>
      <ambient>0.8 0.8 0.8 1</ambient>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.8 0.8 0.8 1</specular>
      </material>
    </visual>
    </link>
  </model>
  
  <include>
    <uri>model://clean_model</uri>
    <pose>0 0 0 0 0 0</pose>
  </include>
  </world>
</sdf>
EOF

# Create ROS 2 launch file for Humble and Fortress
cat > $TEMP_WS/src/stl_model/launch/display_model.py << EOF
#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
  pkg_stl_model = get_package_share_directory('stl_model')
  
  # World file path
  world_file = os.path.join(pkg_stl_model, 'worlds', 'clean_world.sdf')
  
  # Model path for Gazebo
  gazebo_model_path = os.path.join(pkg_stl_model, 'models')
  
  if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] += os.pathsep + gazebo_model_path
  else:
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = gazebo_model_path
  
  # Launch Gazebo Fortress
  gazebo = ExecuteProcess(
    cmd=['ign', 'gazebo', '-r', world_file],
    output='screen'
  )
  
  return LaunchDescription([
    gazebo
  ])
EOF

# Create package.xml for ROS 2
cat > $TEMP_WS/src/stl_model/package.xml << EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>stl_model</name>
  <version>0.0.1</version>
  <description>Package for displaying STL model in Gazebo Fortress</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>Apache-2.0</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  
  <export>
  <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# Create CMakeLists.txt for ROS 2
cat > $TEMP_WS/src/stl_model/CMakeLists.txt << EOF
cmake_minimum_required(VERSION 3.8)
project(stl_model)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install directories
install(DIRECTORY
  launch
  models
  worlds
  DESTINATION share/\${PROJECT_NAME}
)

# Install Python scripts
ament_python_install_package(\${PROJECT_NAME})

ament_package()
EOF

# Create setup.py for Python support
mkdir -p $TEMP_WS/src/stl_model/stl_model
touch $TEMP_WS/src/stl_model/stl_model/__init__.py

cat > $TEMP_WS/src/stl_model/setup.py << EOF
from setuptools import find_packages, setup

package_name = 'stl_model'

setup(
  name=package_name,
  version='0.0.1',
  packages=find_packages(),
  data_files=[
    ('share/ament_index/resource_index/packages',
      ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
  ],
  install_requires=['setuptools'],
  zip_safe=True,
  maintainer='User',
  maintainer_email='user@example.com',
  description='Package for displaying STL model in Gazebo Fortress',
  license='Apache-2.0',
  tests_require=['pytest'],
  entry_points={
    'console_scripts': [
    ],
  },
)
EOF

mkdir -p $TEMP_WS/src/stl_model/resource
touch $TEMP_WS/src/stl_model/resource/stl_model

# Source ROS 2 setup
echo -e "${YELLOW}Sourcing ROS 2 Humble setup...${NC}"
source /opt/ros/humble/setup.bash

# Install additional Python dependencies
echo -e "${YELLOW}Installing Python dependencies...${NC}"
# Ensure python3-venv is installed
if ! dpkg -l | grep -q "ii  python3-venv"; then
  echo -e "${YELLOW}Installing python3-venv...${NC}"
  sudo apt-get update && sudo apt-get install -y python3-venv
fi

# Install system Python packages instead of using a virtual environment
echo -e "${YELLOW}Installing required Python packages...${NC}"
sudo apt-get update && sudo apt-get install -y \
  cython3 \
  python3-empy \
  python3-numpy \
  python3-packaging \
  python3-setuptools \
  python3-pytest \
  python3-pip

# Install Python packages that might not be in Ubuntu repositories
echo -e "${YELLOW}Installing additional Python packages...${NC}"
pip3 install --user pyquaternion

# Build the workspace - exclude Python virtual environment from colcon discovery
cd $TEMP_WS
echo -e "${YELLOW}Building workspace...${NC}"
colcon build --symlink-install --packages-select stl_model

# Source the workspace
source install/setup.bash

# Make launch file executable
chmod +x $TEMP_WS/src/stl_model/launch/display_model.py

# Launch Gazebo with our model
echo -e "${GREEN}Launching Gazebo Fortress with the STL model...${NC}"
# Export the Gazebo model path before launching
export IGN_GAZEBO_RESOURCE_PATH=$TEMP_WS/src/stl_model/models:$IGN_GAZEBO_RESOURCE_PATH
ros2 launch stl_model display_model.py

# Clean up temp workspace
echo -e "${YELLOW}Cleaning up temporary files...${NC}"
rm -rf $TEMP_WS
echo -e "${GREEN}Done!${NC}"

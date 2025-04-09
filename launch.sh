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

# Install Nav2 packages
echo -e "${YELLOW}Installing Nav2 and related packages...${NC}"
sudo apt-get install -y ros-humble-nav2-bringup ros-humble-navigation2 ros-humble-slam-toolbox

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
# Create directories required by CMakeLists.txt
mkdir -p $TEMP_WS/src/stl_model/urdf
mkdir -p $TEMP_WS/src/stl_model/params
mkdir -p $TEMP_WS/src/stl_model/rviz

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
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"></plugin>
    <plugin filename="ignition-gazebo-magnetometer-system" name="ignition::gazebo::systems::Magnetometer"></plugin>
    <plugin filename="ignition-gazebo-pose-publisher-system" name="ignition::gazebo::systems::PosePublisher">
      <publish_link_pose>true</publish_link_pose>
      <publish_model_pose>true</publish_model_pose>
      <publish_nested_model_pose>true</publish_nested_model_pose>
      <publish_collision_pose>false</publish_collision_pose>
      <publish_visual_pose>false</publish_visual_pose>
      <publish_sensor_pose>false</publish_sensor_pose>
      <update_frequency>10</update_frequency>
    </plugin>
    <plugin filename="ignition-gazebo-tf-system" name="ignition::gazebo::systems::TF">
      <update_frequency>10</update_frequency>
    </plugin>
    
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
      <pose>5 5 0 0 0 0</pose>
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
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.conditions import IfCondition
import xacro

def generate_launch_description():
  pkg_stl_model = get_package_share_directory('stl_model')
  
  # World file path
  world_file = os.path.join(pkg_stl_model, 'worlds', 'clean_world.sdf')
  
  # Gazebo & model paths
  gazebo_model_path = os.path.join(pkg_stl_model, 'models')
  
  # URDF file for the robot
  urdf_file = os.path.join(pkg_stl_model, 'urdf', 'simple_robot.urdf.xacro')
  robot_description_config = xacro.process_file(urdf_file)
  robot_desc = robot_description_config.toxml()
  
  # Nav2 parameters file
  nav2_params_path = os.path.join(pkg_stl_model, 'params', 'nav2_params.yaml')
  
  # RViz config file
  rviz_config_file = os.path.join(pkg_stl_model, 'rviz', 'nav2_view.rviz')
  
  # Set Gazebo model path
  if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] += os.pathsep + gazebo_model_path
  else:
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = gazebo_model_path

  # Launch Gazebo Fortress
  gazebo = ExecuteProcess(
    cmd=['ign', 'gazebo', '-r', world_file],
    output='screen'
  )
  
  # Joint state publisher
  joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    parameters=[{'use_sim_time': True}]
  )
  
  # Robot state publisher
  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{'use_sim_time': True, 'robot_description': robot_desc}]
  )
  
  # Spawn the robot in Gazebo
  spawn_robot = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
      '-name', 'simple_robot',
      '-x', '2.0',
      '-y', '2.0',
      '-z', '0.1',
      '-Y', '0.0',
      '-topic', '/robot_description'
    ],
    output='screen',
  )
  
  # Create TF transform for odom to base_link (temporary workaround until proper transforms are established)
  robot_localization = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='odom_to_base_link_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
    output='screen'
  )
  
  # Create initial TF transform for map to odom
  map_to_odom = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='map_to_odom_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    output='screen'
  )
  
  # Launch with delays to ensure proper startup order
  delayed_slam = TimerAction(
    period=5.0,  # Wait 5 seconds before starting SLAM
    actions=[
      # Launch SLAM toolbox
      Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
          {'use_sim_time': True},
          {'base_frame': 'base_link'},
          {'odom_frame': 'odom'},
          {'map_frame': 'map'},
          {'publish_period': 0.5},
          {'max_laser_range': 20.0},
          {'online_async': True}
        ]
      )
    ]
  )
  
  delayed_nav2 = TimerAction(
    period=8.0,  # Wait 8 seconds before starting Nav2
    actions=[
      # Launch Nav2
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          get_package_share_directory('nav2_bringup'),
          '/launch/navigation_launch.py'
        ]),
        launch_arguments={
          'params_file': nav2_params_path,
          'use_sim_time': 'True'
        }.items()
      )
    ]
  )
  
  # Launch RViz2 with navigation view
  rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config_file],
    parameters=[{'use_sim_time': True}],
    output='screen'
  )
  
  # Add dependencies for sequential execution
  spawn_robot_after_gazebo = RegisterEventHandler(
    event_handler=OnProcessStart(
      target_action=gazebo,
      on_start=[spawn_robot],
    )
  )
  
  return LaunchDescription([
    gazebo,
    robot_state_publisher,
    joint_state_publisher,
    spawn_robot_after_gazebo,
    robot_localization,   # Add static TF to bridge odom and base_link
    map_to_odom,          # Add static TF for map to odom
    delayed_slam,         # Start SLAM after a delay
    delayed_nav2,         # Start Nav2 after a delay
    rviz
  ])
EOF

# Create package.xml for ROS 2
cat > $TEMP_WS/src/stl_model/package.xml << EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>stl_model</name>
  <version>0.0.1</version>
  <description>Package for displaying STL model with robot navigation in Gazebo Fortress</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>Apache-2.0</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>navigation2</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>xacro</exec_depend>
  
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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Install directories
install(DIRECTORY
  launch
  models
  worlds
  urdf
  params
  rviz
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
  description='Package for displaying STL model with robot navigation in Gazebo Fortress',
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

# Create a simple robot URDF file
cat > $TEMP_WS/src/stl_model/urdf/simple_robot.urdf.xacro << EOF
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <link name="laser_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Use the ROS 2 Gazebo Fortress syntax for sensors and plugins -->
  <gazebo reference="laser_link">
    <sensor type="gpu_ray" name="lidar">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.15</min>
          <max>12.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="laser_controller" filename="libign_gazebo_sensors.so">
        <ros>
          <namespace>/</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/msg/LaserScan</output_type>
        <frame_name>laser_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo>
    <plugin name="diff_drive" filename="libign_gazebo_diff_drive_system.so">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.4</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <ros>
        <namespace>/</namespace>
        <remapping>cmd_vel:=cmd_vel</remapping>
        <remapping>odom:=odom</remapping>
      </ros>
    </plugin>
  </gazebo>
  
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.2 0" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.2 0" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
EOF

# Create Nav2 parameters file
cat > $TEMP_WS/src/stl_model/params/nav2_params.yaml << EOF
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "differential"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    default_bt_xml_filename: "navigate_w_replanning_time.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_distance_traveled_condition_bt_node

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    controller_plugins: ["FollowPath"]

    # DWB parameters
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.26
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.26
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: True
      stateful: True
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0
      
local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: True
      global_frame: odom
      plugin_names: ["obstacles_laser", "voxel_layer", "inflation_layer"]
      plugin_types: ["nav2_costmap_2d::ObstacleLayer", "nav2_costmap_2d::VoxelLayer", "nav2_costmap_2d::InflationLayer"]
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      inflation_layer:
        cost_scaling_factor: 3.0
      obstacles_laser:
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      voxel_layer:
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        unknown_threshold: 15
        mark_threshold: 0
        observation_sources: scan
        scan:
          data_type: LaserScan
          topic: /scan
          marking: True
          clearing: True
          min_obstacle_height: 0.0
          max_obstacle_height: 2.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
      static_layer:
        map_subscribe_transient_local: True
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: True
      plugin_names: ["static_layer", "obstacles_laser", "inflation_layer"]
      plugin_types: ["nav2_costmap_2d::StaticLayer", "nav2_costmap_2d::ObstacleLayer", "nav2_costmap_2d::InflationLayer"]
      robot_radius: 0.22
      obstacles_laser:
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      static_layer:
        map_subscribe_transient_local: True
      always_send_full_costmap: True

map_server:
  ros__parameters:
    use_sim_time: True
    yaml_filename: "map.yaml"

slam_toolbox:
  ros__parameters:
    use_sim_time: true
    base_frame: base_link
    odom_frame: odom
    map_frame: map
    publish_period: 0.5
    max_laser_range: 20.0
    online_async: true
EOF

# Create RViz configuration file
cat > $TEMP_WS/src/stl_model/rviz/nav2_view.rviz << EOF
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /TF1/Frames1
        - /TF1/Tree1
      Splitter Ratio: 0.5
    Tree Height: 787
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: nav2_rviz_plugins/Navigation 2
    Name: Navigation 2
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description File: ""
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
      Name: RobotModel
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: false
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: false
      Tree:
        {}
      Update Interval: 0
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.009999999776482582
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /scan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 0.699999988079071
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map_updates
      Use Timestamp: false
      Value: true
    - Alpha: 1
      Buffer Length: 1
      Class: rviz_default_plugins/Path
      Color: 25; 255; 0
      Enabled: true
      Head Diameter: 0.30000001192092896
      Head Length: 0.20000000298023224
      Length: 0.30000001192092896
      Line Style: Lines
      Line Width: 0.029999999329447746
      Name: Path
      Offset:
        X: 0
        Y: 0
        Z: 0
      Pose Color: 255; 85; 255
      Pose Style: None
      Radius: 0.029999999329447746
      Shaft Diameter: 0.10000000149011612
      Shaft Length: 0.10000000149011612
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /plan
      Value: true
    - Alpha: 0.699999988079071
      Class: rviz_default_plugins/Map
      Color Scheme: costmap
      Draw Behind: false
      Enabled: true
      Name: Global Costmap
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /global_costmap/costmap
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /global_costmap/costmap_updates
      Use Timestamp: false
      Value: true
    - Alpha: 0.699999988079071
      Class: rviz_default_plugins/Map
      Color Scheme: costmap
      Draw Behind: false
      Enabled: true
      Name: Local Costmap
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /local_costmap/costmap
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /local_costmap/costmap_updates
      Use Timestamp: false
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
    - Class: nav2_rviz_plugins/GoalTool
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.785398006439209
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.785398006439209
    Saved: ~
EOF

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

# Install additional packages for Nav2
echo -e "${YELLOW}Installing additional packages for robot navigation...${NC}"
sudo apt-get update

# Install packages in groups to avoid dependency conflicts
echo -e "${YELLOW}Installing ROS 2 navigation packages...${NC}"
sudo apt-get install -y \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-tf2-ros \
  ros-humble-tf2 \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup

# Don't try to install gazebo-ros-pkgs since it conflicts with Gazebo Fortress
echo -e "${YELLOW}Note: Not installing ros-humble-gazebo-ros-pkgs as it conflicts with Gazebo Fortress${NC}"
echo -e "${YELLOW}Using ros-humble-ros-gz instead for Gazebo Fortress integration${NC}"

# Make sure all the required directories are present before building
echo -e "${YELLOW}Verifying all required directories exist...${NC}"
for dir in urdf params rviz; do
  if [ ! -d "$TEMP_WS/src/stl_model/$dir" ]; then
    echo -e "${YELLOW}Creating missing directory: $TEMP_WS/src/stl_model/$dir${NC}"
    mkdir -p "$TEMP_WS/src/stl_model/$dir"
  fi
done

# Debug information before building
echo -e "${YELLOW}Listing all directories in package:${NC}"
ls -la $TEMP_WS/src/stl_model/

# Launch Gazebo with our model
echo -e "${GREEN}Launching Gazebo Fortress with the STL model...${NC}"
# Export the Gazebo model path before launching
export IGN_GAZEBO_RESOURCE_PATH=$TEMP_WS/src/stl_model/models:$IGN_GAZEBO_RESOURCE_PATH
ros2 launch stl_model display_model.py

# Clean up temp workspace
echo -e "${YELLOW}Cleaning up temporary files...${NC}"
rm -rf $TEMP_WS
echo -e "${GREEN}Done!${NC}"


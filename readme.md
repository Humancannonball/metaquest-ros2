# ROS 2 Humble with Gazebo Fortress STL Model Launcher

This utility automatically installs and launches ROS 2 Humble with Gazebo Fortress, and loads an STL file called `clean.stl` into the simulation environment. It's designed to work in both regular Linux environments and inside a distrobox container.

## All-in-One Solution

This script handles:
1. Installation of all prerequisites (if not already installed)
   - ROS 2 Humble
   - Gazebo Fortress
   - ROS-Gazebo integration packages
   - Required system dependencies
2. Creation of a temporary ROS 2 workspace with the STL model
3. Building and launching the simulation

## Prerequisites

- Ubuntu 22.04 (or compatible Linux distribution)
- Internet connection (for installation if needed)
- `clean.stl` file in the same directory as the launch script
- Sudo privileges (for installation if needed)

## Usage

1. Make sure the script is executable:

```bash
chmod +x launch.sh
```

2. Run the script:

```bash
./launch.sh
```

3. If running inside a distrobox, you can enter the container and run the script:

```bash
distrobox enter <container-name>
./launch.sh
```

## What the Script Does

### Installation Phase
- Checks and installs required system packages
- Checks and installs ROS 2 Humble if not present
- Checks and installs Gazebo Fortress if not present
- Installs ROS 2 - Gazebo integration packages

### Setup Phase
- Checks if the STL file exists
- Creates a temporary ROS 2 workspace and package structure
- Sets up the necessary configuration files
- Installs required Python dependencies

### Launch Phase
- Builds the workspace
- Launches Gazebo Fortress with the STL model loaded
- Cleans up temporary files after Gazebo is closed

## Customization

- To use a different STL file, either rename your file to `clean.stl` or modify the `STL_FILE` variable in the script
- Adjust model properties (scale, position, etc.) by editing the model.sdf section in the script
- Change the Gazebo world settings in the clean_world.sdf section

## Troubleshooting

- If you see permission errors, make sure you have sudo privileges
- If package installation fails, check your internet connection
- If the STL file cannot be found, ensure it's in the same directory as the script
- If the simulation crashes, your STL file might have issues - try a simpler model
- For rendering issues, ensure your graphics drivers are up-to-date

## Additional Notes for Distrobox Usage
When running in a distrobox container, ensure that all required packages (like Python, build tools, etc.) are installed inside the container. Apart from that, usage remains the same.
#!/bin/bash

# Ask user for map name
read -p "Enter map name: " MAP_NAME

# Directories and file paths
SAVE_DIR="/home/mr_robot/data/maps"
LOCALIZATION_LAUNCH="/home/mr_robot/devel_ws/install/rom2109_carto/share/rom2109_carto/launch/localization.launch.py"
NAV2_PARAMS="/home/mr_robot/devel_ws/install/rom2109_nav2/share/rom2109_nav2/config/nav2_params.yaml"
NAV2_LAUNCH="/home/mr_robot/devel_ws/install/rom2109_nav2/share/rom2109_nav2/launch/navigation_localization_composable.launch.py"

# Ensure save directory exists
mkdir -p "$SAVE_DIR"

# 1️⃣ Save Cartographer PBSTREAM
echo "Saving PBSTREAM to $SAVE_DIR/${MAP_NAME}.pbstream..."
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
"{filename: '${SAVE_DIR}/${MAP_NAME}.pbstream'}"

sleep 2

# 2️⃣ Convert PBSTREAM → PGM + YAML
echo "Converting PBSTREAM to PGM + YAML..."
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
    -pbstream_filename "${SAVE_DIR}/${MAP_NAME}.pbstream" \
    -map_filestem "${SAVE_DIR}/${MAP_NAME}"

# 3️⃣ Update file references
echo "Updating map references in launch/config files..."

# Update PBSTREAM path in localization.launch.py
sed -i "s|maps/.*\.pbstream|maps/${MAP_NAME}.pbstream|g" "$LOCALIZATION_LAUNCH"

# Update YAML path in nav2_params.yaml
sed -i "s|maps/.*\.yaml|maps/${MAP_NAME}.yaml|g" "$NAV2_PARAMS"

# Update YAML path in navigation_localization_composable.launch.py
sed -i "s|/home/mr_robot/data/maps/.*\.yaml|/home/mr_robot/data/maps/${MAP_NAME}.yaml|g" "$NAV2_LAUNCH"

echo "✅ Map saved and file paths updated!"
echo "PBSTREAM: ${SAVE_DIR}/${MAP_NAME}.pbstream"
echo "YAML: ${SAVE_DIR}/${MAP_NAME}.yaml"

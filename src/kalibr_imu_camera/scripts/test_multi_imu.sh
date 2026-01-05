#!/bin/bash
# Multi-IMU Calibration Test Script
# This script demonstrates how to use the multi-IMU calibration functionality

set -e

echo "================================================"
echo "Kalibr ROS2 - Multi-IMU Calibration Test"
echo "================================================"
echo ""

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS2 is not sourced. Please run:"
    echo "  source /opt/ros/humble/setup.bash"
    echo "  source /path/to/kalibr_ros2/install/setup.bash"
    exit 1
fi

# Check if kalibr_imu_camera is installed
if ! ros2 pkg list | grep -q kalibr_imu_camera; then
    echo "Error: kalibr_imu_camera package not found"
    echo "Please build the package first:"
    echo "  cd kalibr_ros2"
    echo "  colcon build --packages-select kalibr_imu_camera"
    exit 1
fi

echo "✓ Environment check passed"
echo ""

# Show available tools
echo "Available calibration tools:"
echo "  1. kalibr_calibrate_cameras - Camera system calibration"
echo "  2. kalibr_calibrate_imu_camera - IMU-Camera calibration (supports multi-IMU)"
echo ""

# Check for example configuration files
PKG_SHARE=$(ros2 pkg prefix kalibr_imu_camera)/share/kalibr_imu_camera

if [ -f "$PKG_SHARE/config/imu0_example.yaml" ]; then
    echo "✓ Found example configuration files in: $PKG_SHARE/config/"
    echo "  - imu0_example.yaml (reference IMU)"
    echo "  - imu1_example.yaml (second IMU)"
    echo "  - target_april_example.yaml (AprilGrid target)"
    echo ""
else
    echo "⚠ Warning: Example configuration files not found"
    echo ""
fi

echo "================================================"
echo "Multi-IMU Calibration Usage Examples"
echo "================================================"
echo ""

echo "Example 1: Dual-IMU calibration with calibrated models"
echo "-------------------------------------------------------"
cat << 'EOF'
ros2 run kalibr_imu_camera kalibr_calibrate_imu_camera \
  --bag multi_imu_camera_bag \
  --cams camchain.yaml \
  --imu imu0.yaml imu1.yaml \
  --imu-models calibrated calibrated \
  --imu-delay-by-correlation \
  --target aprilgrid.yaml \
  --show-extraction
EOF
echo ""

echo "Example 2: Using launch file for dual-IMU calibration"
echo "------------------------------------------------------"
cat << 'EOF'
ros2 launch kalibr_imu_camera calibrate_multi_imu.launch.py \
  bagfile:=multi_imu_camera_bag \
  camchain:=camchain.yaml \
  imu0:=imu0.yaml \
  imu1:=imu1.yaml \
  target_yaml:=aprilgrid.yaml \
  imu_models:='calibrated calibrated'
EOF
echo ""

echo "Example 3: Triple-IMU calibration"
echo "----------------------------------"
cat << 'EOF'
ros2 run kalibr_imu_camera kalibr_calibrate_imu_camera \
  --bag multi_imu_camera_bag \
  --cams camchain.yaml \
  --imu imu0.yaml imu1.yaml imu2.yaml \
  --imu-models calibrated calibrated calibrated \
  --imu-delay-by-correlation \
  --target aprilgrid.yaml \
  --show-extraction
EOF
echo ""

echo "Example 4: Mixed IMU models (calibrated + scale-misalignment)"
echo "--------------------------------------------------------------"
cat << 'EOF'
ros2 run kalibr_imu_camera kalibr_calibrate_imu_camera \
  --bag multi_imu_camera_bag \
  --cams camchain.yaml \
  --imu imu0.yaml imu1.yaml \
  --imu-models calibrated scale-misalignment \
  --imu-delay-by-correlation \
  --target aprilgrid.yaml \
  --show-extraction
EOF
echo ""

echo "================================================"
echo "Key Points for Multi-IMU Calibration"
echo "================================================"
echo "1. The FIRST IMU (imu0) is the REFERENCE IMU"
echo "2. All other IMUs are calibrated relative to imu0"
echo "3. Use --imu-delay-by-correlation for non-hardware-synced IMUs"
echo "4. Each IMU needs its own YAML configuration file"
echo "5. Number of IMU YAMLs must match number of IMU models"
echo ""

echo "================================================"
echo "For detailed documentation, see:"
echo "================================================"
echo "  - README.md: General overview"
echo "  - MULTI_IMU_CALIBRATION.md: Detailed multi-IMU guide"
echo "  - CAMERA_CALIBRATION.md: Camera calibration guide"
echo ""

echo "Test script completed successfully!"

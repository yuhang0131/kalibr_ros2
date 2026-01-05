#!/bin/bash
# Script to build Kalibr ROS2 workspace in correct dependency order

set -e  # Exit on error

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Kalibr ROS2 Workspace Build Script${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Function to build a specific package
build_package() {
    local pkg_name=$1
    echo -e "${YELLOW}Building package: ${pkg_name}${NC}"
    
    if colcon build --packages-select "${pkg_name}" --cmake-args -DCMAKE_BUILD_TYPE=Release; then
        echo -e "${GREEN}✓ Successfully built ${pkg_name}${NC}"
        return 0
    else
        echo -e "${RED}✗ Failed to build ${pkg_name}${NC}"
        return 1
    fi
}

# Function to build multiple packages in one layer
build_layer() {
    local layer_num=$1
    shift
    local packages=("$@")
    
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Building Layer ${layer_num}${NC}"
    echo -e "${GREEN}========================================${NC}"
    
    for pkg in "${packages[@]}"; do
        build_package "${pkg}" || exit 1
    done
    
    echo ""
}

# Check if we should build all or specific layers
BUILD_ALL=true
START_LAYER=1
END_LAYER=10

if [ "$1" == "--layer" ] && [ -n "$2" ]; then
    BUILD_ALL=false
    START_LAYER=$2
    END_LAYER=$2
    echo "Building only layer $START_LAYER"
elif [ "$1" == "--from-layer" ] && [ -n "$2" ]; then
    BUILD_ALL=false
    START_LAYER=$2
    echo "Building from layer $START_LAYER to end"
fi

# Layer 1: Basic utilities (no internal dependencies)
if [ $START_LAYER -le 1 ] && [ $END_LAYER -ge 1 ]; then
    build_layer 1 sm_common sm_random sm_logging python_module aslam_time ethz_apriltag2
fi

# Layer 2: Depends on Layer 1
if [ $START_LAYER -le 2 ] && [ $END_LAYER -ge 2 ]; then
    build_layer 2 sm_boost sm_timing sm_opencv sm_property_tree sm_matrix_archive
fi

# Layer 3
if [ $START_LAYER -le 3 ] && [ $END_LAYER -ge 3 ]; then
    build_layer 3 sm_eigen numpy_eigen sparse_block_matrix
fi

# Layer 4
if [ $START_LAYER -le 4 ] && [ $END_LAYER -ge 4 ]; then
    build_layer 4 sm_kinematics aslam_backend
fi

# Layer 5
if [ $START_LAYER -le 5 ] && [ $END_LAYER -ge 5 ]; then
    build_layer 5 aslam_backend_expressions aslam_cameras bsplines sm_python incremental_calibration
fi

# Layer 6
if [ $START_LAYER -le 6 ] && [ $END_LAYER -ge 6 ]; then
    build_layer 6 aslam_backend_python aslam_splines aslam_cv_serialization aslam_imgproc bsplines_python aslam_cameras_april incremental_calibration_python
fi

# Layer 7
if [ $START_LAYER -le 7 ] && [ $END_LAYER -ge 7 ]; then
    build_layer 7 aslam_cv_backend aslam_cv_python aslam_splines_python
fi

# Layer 8
if [ $START_LAYER -le 8 ] && [ $END_LAYER -ge 8 ]; then
    build_layer 8 aslam_cv_error_terms
fi

# Layer 9
if [ $START_LAYER -le 9 ] && [ $END_LAYER -ge 9 ]; then
    build_layer 9 aslam_cv_backend_python
fi

# Layer 10: Top-level application
if [ $START_LAYER -le 10 ] && [ $END_LAYER -ge 10 ]; then
    build_layer 10 kalibr_imu_camera
fi

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Build Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Source the workspace:"
echo "  source install/setup.bash"
echo ""
echo "Run calibration:"
echo "  ros2 launch kalibr_imu_camera calibrate_imu_camera.launch.py bagfile:=/path/to/bag ..."

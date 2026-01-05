cmake_minimum_required(VERSION 3.5)
project(kalibr_imu_camera)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosbag2_py REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Find internal kalibr dependencies
find_package(sm_common REQUIRED)
find_package(sm_python REQUIRED)
find_package(aslam_cv_python REQUIRED)
find_package(aslam_cv_backend_python REQUIRED)
find_package(aslam_backend_python REQUIRED)
find_package(aslam_backend REQUIRED)
find_package(aslam_backend_expressions REQUIRED)
find_package(aslam_splines_python REQUIRED)
find_package(aslam_cameras_april REQUIRED)
find_package(incremental_calibration_python REQUIRED)
find_package(numpy_eigen REQUIRED)
find_package(python_module REQUIRED)
find_package(sparse_block_matrix REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization python thread filesystem regex)
find_package(Python3 REQUIRED COMPONENTS Development NumPy)

# Build errorterms C++ library
include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(kalibr_errorterms SHARED
  src/EuclideanError.cpp
  src/GyroscopeError.cpp
  src/AccelerometerError.cpp
)

set_target_properties(kalibr_errorterms PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

ament_target_dependencies(kalibr_errorterms
  aslam_backend
  aslam_backend_expressions
  sparse_block_matrix
  sm_common
)

install(TARGETS kalibr_errorterms
  EXPORT kalibr_errortermsTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Build Python bindings for errorterms
add_library(libkalibr_errorterms_python SHARED
  src/module.cpp
)

target_include_directories(libkalibr_errorterms_python PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
)

target_link_libraries(libkalibr_errorterms_python
  kalibr_errorterms
  -Wl,--no-as-needed
  ${Boost_LIBRARIES}
  boost_regex
  cholmod
  spqr
  -Wl,--as-needed
  ${Python3_LIBRARIES}
)

ament_target_dependencies(libkalibr_errorterms_python
  numpy_eigen
  aslam_backend
  aslam_backend_expressions
  python_module
)

set_target_properties(libkalibr_errorterms_python PROPERTIES
  PREFIX ""
  OUTPUT_NAME "libkalibr_errorterms_python"
)

install(TARGETS libkalibr_errorterms_python
  LIBRARY DESTINATION local/lib/python3.10/dist-packages
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR python/kalibr_imu_camera
)

ament_python_install_package(kalibr_common
  PACKAGE_DIR python/kalibr_common
)

ament_python_install_package(kalibr_camera_calibration
  PACKAGE_DIR python/kalibr_camera_calibration
)

ament_python_install_package(kalibr_imu_camera_calibration
  PACKAGE_DIR python/kalibr_imu_camera_calibration
)

ament_python_install_package(kalibr_errorterms
  PACKAGE_DIR python/kalibr_errorterms
)

# Install Python executables
install(PROGRAMS
  python/kalibr_calibrate_imu_camera
  scripts/kalibr_calibrate_cameras
  DESTINATION lib/${PROJECT_NAME}
)

# Create launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

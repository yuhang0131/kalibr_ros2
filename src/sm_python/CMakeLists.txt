cmake_minimum_required(VERSION 3.5)
project(sm_python)

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
find_package(sm_common REQUIRED)
find_package(numpy_eigen REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(sm_timing REQUIRED)
find_package(sm_logging REQUIRED)
find_package(sm_matrix_archive REQUIRED)
find_package(sm_property_tree REQUIRED)
find_package(python_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization python thread filesystem regex)
find_package(Python3 REQUIRED COMPONENTS Development NumPy)

add_definitions(-D__STRICT_ANSI__)
add_compile_options(-fpermissive)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
)

# Create Python extension module
add_library(${PROJECT_NAME} SHARED
  src/module.cpp
  src/export_rotational_kinematics.cpp
  src/export_rotations.cpp
  src/export_transformations.cpp
  src/export_quaternion_algebra.cpp
  src/export_homogeneous.cpp
  src/exportTransformation.cpp
  src/exportHomogeneousPoint.cpp
  src/exportTimestampCorrector.cpp
  src/exportPropertyTree.cpp
  src/exportEigen.cpp
  src/exportUncertainVector.cpp
  src/exportMatrixArchive.cpp
  src/export_kinematics_property_tree.cpp
  src/export_eigen_property_tree.cpp
  src/Logging.cpp
  src/exportNsecTime.cpp
  src/random.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  -Wl,--no-as-needed
  ${Boost_LIBRARIES}
  ${Python3_LIBRARIES}
  boost_regex
  -Wl,--as-needed
)

ament_target_dependencies(${PROJECT_NAME}
  sm_common numpy_eigen sm_kinematics sm_timing sm_logging
  sm_matrix_archive sm_property_tree python_module
)

# Python extension modules should not have "lib" prefix
set_target_properties(${PROJECT_NAME} PROPERTIES
  PREFIX ""
  OUTPUT_NAME "libsm_python"
)

# Install Python source files first
ament_python_install_package(sm
  PACKAGE_DIR python/sm
)

# Install the Python module (.so) to the same location as Python files
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION local/lib/python3.10/dist-packages/sm
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(sm_common numpy_eigen sm_kinematics sm_timing sm_logging sm_matrix_archive sm_property_tree python_module Boost Python3 Eigen3)
ament_export_include_directories(include)

ament_package()

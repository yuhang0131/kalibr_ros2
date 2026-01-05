cmake_minimum_required(VERSION 3.5)
project(aslam_cameras_april)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated-copy)
endif()

find_package(ament_cmake REQUIRED)
find_package(aslam_cameras REQUIRED)
find_package(ethz_apriltag2 REQUIRED)
find_package(sm_opencv REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_logging REQUIRED)
find_package(sm_boost REQUIRED)
find_package(sm_eigen REQUIRED)
find_package(sm_python REQUIRED)
find_package(numpy_eigen REQUIRED)
find_package(python_module REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization python thread filesystem regex)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

# C++ library
add_library(${PROJECT_NAME} SHARED
  src/GridCalibrationTargetAprilgrid.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  aslam_cameras
  ethz_apriltag2
  sm_opencv
  sm_common
  sm_logging
  sm_boost
  sm_eigen
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  aslam_cameras
  ethz_apriltag2
  sm_opencv
  sm_common
  sm_logging
  sm_boost
  sm_eigen
  Boost
  Eigen3
  OpenCV
)

# Python bindings
add_library(${PROJECT_NAME}_python SHARED
  src/module.cpp
)

target_include_directories(${PROJECT_NAME}_python PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_python
  ${PROJECT_NAME}
  -Wl,--no-as-needed
  ${Boost_LIBRARIES}
  boost_regex
  cholmod
  spqr
  -Wl,--as-needed
  ${Python3_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_python
  numpy_eigen
  sm_python
  aslam_cameras
  python_module
)

set_target_properties(${PROJECT_NAME}_python PROPERTIES
  PREFIX ""
  OUTPUT_NAME "libaslam_cameras_april_python"
)

ament_python_install_package(aslam_cameras_april
  PACKAGE_DIR python/aslam_cameras_april
)

install(TARGETS ${PROJECT_NAME}_python
  LIBRARY DESTINATION local/lib/python3.10/dist-packages/aslam_cameras_april
)

ament_package()

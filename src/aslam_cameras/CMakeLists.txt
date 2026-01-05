cmake_minimum_required(VERSION 3.5)
project(aslam_cameras)

ADD_DEFINITIONS(-DASLAM_USE_ROS)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated-copy)
endif()

find_package(ament_cmake REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_eigen REQUIRED)
find_package(sm_boost REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(sm_logging REQUIRED)
find_package(aslam_time REQUIRED)
find_package(sm_opencv REQUIRED)
find_package(sm_property_tree REQUIRED)
find_package(sm_random REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization filesystem thread)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
  src/CameraGeometryBase.cpp
  src/GlobalShutter.cpp
  src/RollingShutter.cpp
  src/NoMask.cpp
  src/NoDistortion.cpp
  src/RadialTangentialDistortion.cpp
  src/EquidistantDistortion.cpp
  src/FovDistortion.cpp
  src/ImageMask.cpp
  src/GridCalibrationTargetObservation.cpp
  src/GridCalibrationTargetBase.cpp
  src/GridCalibrationTargetCheckerboard.cpp
  src/GridCalibrationTargetCirclegrid.cpp
  src/GridDetector.cpp
  src/Triangulation.cpp
  src/FrameBase.cpp
  src/KeypointBase.cpp
  src/DescriptorBase.cpp
  src/BackProjection.cpp
  src/Landmark.cpp
  src/Image.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  sm_common sm_eigen sm_boost sm_kinematics sm_logging aslam_time sm_opencv sm_property_tree sm_random
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
ament_export_dependencies(sm_common sm_eigen sm_boost sm_kinematics sm_logging aslam_time sm_opencv sm_property_tree sm_random Boost Eigen3 OpenCV)
ament_export_include_directories(include)

ament_package()

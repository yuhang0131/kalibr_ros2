cmake_minimum_required(VERSION 3.5)
project(aslam_cv_backend)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated-copy)
endif()

find_package(ament_cmake REQUIRED)
find_package(aslam_backend REQUIRED)
find_package(aslam_backend_expressions REQUIRED)
find_package(aslam_cameras REQUIRED)
find_package(aslam_splines REQUIRED)
find_package(bsplines REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(sm_property_tree REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization filesystem thread)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/GridCalibrationTargetDesignVariableContainer.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME} aslam_backend aslam_backend_expressions aslam_cameras aslam_splines bsplines sm_kinematics sm_property_tree)

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
ament_export_include_directories(include)

ament_package()

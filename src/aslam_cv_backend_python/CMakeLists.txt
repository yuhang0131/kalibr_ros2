cmake_minimum_required(VERSION 3.5)
project(aslam_cv_backend_python)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated-copy)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(numpy_eigen REQUIRED)
find_package(sm_python REQUIRED)
find_package(aslam_backend REQUIRED)
find_package(aslam_backend_expressions REQUIRED)
find_package(aslam_backend_python REQUIRED)
find_package(aslam_cameras REQUIRED)
find_package(aslam_cv_error_terms REQUIRED)
find_package(aslam_cv_backend REQUIRED)
find_package(aslam_cv_python REQUIRED)
find_package(aslam_splines REQUIRED)
find_package(bsplines REQUIRED)
find_package(sparse_block_matrix REQUIRED)
find_package(aslam_time REQUIRED)
find_package(python_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization python thread filesystem regex)
find_package(Python3 REQUIRED COMPONENTS Development NumPy)
find_package(OpenCV REQUIRED)

add_definitions(-D__STRICT_ANSI__)
add_compile_options(-fpermissive)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/module.cpp  
  src/GridCalibration.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  -Wl,--no-as-needed
  ${Boost_LIBRARIES}
  boost_regex
  cholmod
  spqr
  -Wl,--as-needed
  ${Python3_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  numpy_eigen sm_python aslam_backend aslam_backend_expressions aslam_backend_python
  aslam_cameras aslam_cv_error_terms aslam_cv_backend aslam_cv_python 
  aslam_splines bsplines sparse_block_matrix aslam_time python_module
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  PREFIX ""
  OUTPUT_NAME "libaslam_cv_backend_python"
)

ament_python_install_package(aslam_cv_backend
  PACKAGE_DIR python/aslam_cv_backend
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION local/lib/python3.10/dist-packages/aslam_cv_backend
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(numpy_eigen sm_python aslam_backend aslam_backend_expressions aslam_backend_python aslam_cameras aslam_cv_error_terms aslam_cv_backend aslam_cv_python aslam_splines bsplines sparse_block_matrix aslam_time python_module Boost Python3 Eigen3 OpenCV)
ament_export_include_directories(include)

ament_package()

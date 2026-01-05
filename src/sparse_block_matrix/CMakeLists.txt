cmake_minimum_required(VERSION 3.5)
project(sparse_block_matrix)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization)

# Find SuiteSparse - use direct paths as pkg-config files may not exist
set(CHOLMOD_INCLUDE_DIRS /usr/include/suitesparse)
set(CHOLMOD_LIBRARIES cholmod spqr)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${CHOLMOD_INCLUDE_DIRS}
  ${SUITESPARSE_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/matrix_structure.cpp
  src/sparse_helper.cpp
  src/marginal_covariance_cholesky.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${CHOLMOD_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${CHOLMOD_LIBRARIES}
  ${Boost_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME} sm_common sm_eigen Boost)

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
ament_export_dependencies(sm_common sm_eigen Eigen3)
ament_export_include_directories(include)

ament_package()

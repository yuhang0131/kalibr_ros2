cmake_minimum_required(VERSION 3.5)
project(sm_common)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Add library
include_directories(include ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/progress_info.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
)

target_compile_options(${PROJECT_NAME} PUBLIC
  -D__STRICT_ANSI__
  -Wno-deprecated-copy
)

# Install library
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Export targets
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost)
ament_export_include_directories(include)

ament_package()

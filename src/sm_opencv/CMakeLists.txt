cmake_minimum_required(VERSION 3.5)
project(sm_opencv)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sm_common REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Header-only library
install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(sm_common Boost)
ament_export_include_directories(include)

ament_package()

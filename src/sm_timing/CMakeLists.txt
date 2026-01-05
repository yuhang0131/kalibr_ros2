cmake_minimum_required(VERSION 3.5)
project(sm_timing)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_random REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(include ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/Timer.cpp
  src/NsecTimeUtilities.cpp
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

target_compile_options(${PROJECT_NAME} PUBLIC -D__STRICT_ANSI__)

ament_target_dependencies(${PROJECT_NAME} sm_common sm_random)

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
ament_export_dependencies(sm_common sm_random Boost)
ament_export_include_directories(include)

ament_package()

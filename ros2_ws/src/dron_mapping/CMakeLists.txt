cmake_minimum_required(VERSION 3.8)
project(dron_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(dron_interfaces REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Core mapping library
add_library(dron_mapping_core
  src/visibility_buffer.cpp
  src/thermal_projector.cpp
  src/observation_store.cpp
  src/rtabmap_adapter.cpp
)
target_link_libraries(dron_mapping_core
  ${OpenCV_LIBRARIES}
)

# Nodes
add_executable(thermal_mapping_node
  src/thermal_mapping_node.cpp
)
target_link_libraries(thermal_mapping_node
  dron_mapping_core
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(thermal_mapping_node
  rclcpp
  sensor_msgs
  geometry_msgs
  message_filters
  cv_bridge
  dron_interfaces
)

add_executable(map_snapshot_server
  src/map_snapshot_server.cpp
)
target_link_libraries(map_snapshot_server
  dron_mapping_core
)
ament_target_dependencies(map_snapshot_server
  rclcpp
  dron_interfaces
)

install(TARGETS
  dron_mapping_core
  thermal_mapping_node
  map_snapshot_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(dron_mapping_core)
ament_package()

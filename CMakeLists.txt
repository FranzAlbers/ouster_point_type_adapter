cmake_minimum_required(VERSION 3.10)
project(ouster_point_type_adapter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ouster_ros REQUIRED)
find_package(autoware_point_types REQUIRED)

# ==== Options ====
add_compile_options(-Wall -Wextra)
if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
option(CMAKE_POSITION_INDEPENDENT_CODE "Build position independent code." ON)

add_library(${PROJECT_NAME} SHARED
  src/ouster_point_type_adapter_component.cpp
)

include_directories(
  include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ouster_ros_INCLUDE_DIRS}
)

include_directories(SYSTEM 
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  pcl_conversions
  ouster_ros
  autoware_point_types
)

rclcpp_components_register_nodes(${PROJECT_NAME} PLUGIN "ouster_point_type_adapter::OusterPointTypeAdapter" EXECUTABLE ouster_point_type_adapter)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_package()

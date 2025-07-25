cmake_minimum_required(VERSION 3.16)
project(sensor_bridge)

set (VESIAPI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/V-ESI-API)
set(exe_name SensorBridgeNode)

set(DCMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_STANDARD 99)

# find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(MSG_DEPS sensor_msgs)

include_directories(
  include
)

set(dependencies
  rclcpp
  sensor_msgs
  rosidl_default_generators
)

install(DIRECTORY include/
  DESTINATION include/
)

add_executable(${exe_name} src/bridge.cpp include/bridge.h)

link_directories(${VESIAPI_DIR}/lib/)

ament_target_dependencies(${exe_name} ${dependencies})

target_link_libraries(
  ${exe_name}
  ${VESIAPI_DIR}/lib/linux/libVESIAPI.so
)

target_include_directories(${exe_name}
  PUBLIC include/sensor-bridge
  PUBLIC include/V-ESI-API/include
)

install(TARGETS ${exe_name}
  DESTINATION lib/${PROJECT_NAME}
)

# Install the VESI API library into the install directory
install(FILES
  ${VESIAPI_DIR}/lib/linux/libVESIAPI.so
  DESTINATION lib
)

# Runtime link the VESI API path (rather than a static path)
set_target_properties(
  ${exe_name} PROPERTIES
  INSTALL_RPATH "$ORIGIN/.."
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

cmake_minimum_required(VERSION 3.8)
project(unitree_hardware_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/go_motor_control/include
)

link_directories(lib)

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(EXTRA_LIBS libUnitreeMotorSDK_Arm64.so)
else()
  set(EXTRA_LIBS libUnitreeMotorSDK_Linux64.so)
endif()

add_library(${PROJECT_NAME} SHARED
  src/unitree_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/go_motor_control/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME}
  hardware_interface::hardware_interface
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${EXTRA_LIBS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  PLUGINLIB__DISABLE_BOOST_FUNCTIONS
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES
  lib/libUnitreeMotorSDK_Arm64.so
  lib/libUnitreeMotorSDK_Linux64.so
  DESTINATION lib
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
)

install(PROGRAMS scripts/send_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(
  hardware_interface ${CMAKE_CURRENT_SOURCE_DIR}/unitree_hardware_interface.xml
)

ament_export_dependencies(
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()

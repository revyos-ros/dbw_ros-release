cmake_minimum_required(VERSION 3.5)
project(dataspeed_ulc_can)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(dataspeed_dbw_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dataspeed_ulc_msgs REQUIRED)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(STATUS "Enabling coverage testing")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

add_library(${PROJECT_NAME} SHARED
  "src/UlcNode.cpp"
)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp_components"
  "rclcpp"
  "dataspeed_dbw_common"
  "std_msgs"
  "can_msgs"
  "geometry_msgs"
  "dataspeed_ulc_msgs"
)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "${PROJECT_NAME}::UlcNode"
  EXECUTABLE ulc_node
)

ament_export_include_directories(include)
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY scripts
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  # find_package(roslib REQUIRED)
  # find_package(rostest REQUIRED)
  
  # Test node
  # add_rostest_gtest(${PROJECT_NAME}_test_node test_ulc_node.test test_ulc_node.cpp)
  # add_dependencies(${PROJECT_NAME}_test_node ${catkin_EXPORTED_TARGETS})
  # target_link_libraries(${PROJECT_NAME}_test_node ${catkin_LIBRARIES})
  
  # # Test nodelet
  # add_rostest_gtest(${PROJECT_NAME}_test_nodelet test_ulc_nodelet.test test_ulc_node.cpp)
  # add_dependencies(${PROJECT_NAME}_test_nodelet ${catkin_EXPORTED_TARGETS})
  # target_link_libraries(${PROJECT_NAME}_test_nodelet ${catkin_LIBRARIES})
  
  # # Test config frequency parameter (for ROS distributions newer than Indigo)
  # if($ENV{ROS_DISTRO} STRGREATER indigo)
  #   add_rostest(param_cfg_freq.test ARGS config_freq:=2  param_value:=5)
  #   add_rostest(param_cfg_freq.test ARGS config_freq:=20 param_value:=20)
  #   add_rostest(param_cfg_freq.test ARGS config_freq:=99 param_value:=50)
  # endif()
endif()

ament_package()
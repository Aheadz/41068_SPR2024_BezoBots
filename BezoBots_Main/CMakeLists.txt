cmake_minimum_required(VERSION 3.8)
project(bezobots)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
          
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(turtlebot3_navigation2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)


# Add a single executable
add_executable(robo_main src/main.cpp src/robo_s2.cpp src/robo_s3.cpp)

ament_target_dependencies(
  robo_main 
  rclcpp 
  rclcpp_action 
  std_msgs 
  sensor_msgs 
  nav_msgs 
  tf2
  tf2_geometry_msgs
  tf2_ros
  OpenCV
)

target_include_directories(robo_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link OpenCV libraries to your executable
target_link_libraries(robo_main ${OpenCV_LIBRARIES})

target_compile_features(robo_main PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
  robo_main
  DESTINATION lib/${PROJECT_NAME})

# Ensure the headers are installed
install(DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.hpp")

# Install the maps directory (without nesting)
install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY launch models config urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)

  set(
    _dirs_to_check

    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/launch
  )

  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright(${_dirs_to_check})

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(${_dirs_to_check} FILTERS -runtime/int)

  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck(${_dirs_to_check})

  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake(${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt)

  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint(${_dirs_to_check} ${CMAKE_CURRENT_SOURCE_DIR})

  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify(${_dirs_to_check} LANGUAGE C++)
  
endif()

# this must be added so launch files can be used from CLI
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_geometry_msgs)
ament_package()

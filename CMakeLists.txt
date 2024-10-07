cmake_minimum_required(VERSION 3.8)
project(41068_SPR2024_BezoBots)

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
find_package(OpenCV REQUIRED)

include_directories(include)


# Add a single executable
add_executable(robo_s2 src/main.cpp src/robo_s2.cpp)

ament_target_dependencies(
  robo_s2 
  rclcpp 
  rclcpp_action 
  std_msgs 
  sensor_msgs 
  nav_msgs 
  tf2
)

target_include_directories(robo_s2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link OpenCV libraries to your executable
target_link_libraries(robo_s2 ${OpenCV_LIBRARIES})

target_compile_features(robo_s2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
  robo_s2
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

ament_package()

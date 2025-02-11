cmake_minimum_required(VERSION 3.8)
project(linefollower_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenCV REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(line_follower script/image_cv.cpp)
ament_target_dependencies(line_follower rclcpp sensor_msgs cv_bridge OpenCV)

install(PROGRAMS
  script/image_cv.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch meshes urdf config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
TARGETS line_follower 
DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.5)
project(nero_drone)

# ---------------- Dependencies ----------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# ---------------- Executables C++ ----------------
# Hello World C++
add_executable(hello_cpp src/hello_cpp.cpp)
ament_target_dependencies(hello_cpp rclcpp)

# AprilTag detector con webcam
add_executable(apriltag_node src/apriltag_node.cpp)
ament_target_dependencies(apriltag_node rclcpp OpenCV)
target_link_libraries(apriltag_node apriltag ${OpenCV_LIBS})

# AprilTag detector para la cámara del Bebop
add_executable(beboptag src/beboptag.cpp)
ament_target_dependencies(beboptag
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)
target_link_libraries(beboptag apriltag ${OpenCV_LIBS})

# ---------------- Install ----------------
install(TARGETS
  hello_cpp
  apriltag_node
  beboptag
  DESTINATION lib/${PROJECT_NAME}
)

# Python nodes
install(PROGRAMS
  nero_drone/hello_py.py
  nero_drone/apriltag.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

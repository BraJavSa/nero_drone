cmake_minimum_required(VERSION 3.5)
project(nero_drone)

# ---------------- Dependencies ----------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# ---------------- Executables C++ ----------------
# Hello World C++
add_executable(hello_cpp src/hello_cpp.cpp)
ament_target_dependencies(hello_cpp rclcpp)

# AprilTag detector C++
add_executable(apriltag_node src/apriltag_node.cpp)
ament_target_dependencies(apriltag_node rclcpp OpenCV)

# 👇 fuerza a linkar con la libapriltag
target_link_libraries(apriltag_node apriltag ${OpenCV_LIBS})

# ---------------- Install ----------------
install(TARGETS
  hello_cpp
  apriltag_node
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

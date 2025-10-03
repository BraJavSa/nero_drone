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
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ---------------- Includes ----------------
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# ---------------- Executables C++ ----------------

add_executable(hello_cpp src/hello_cpp.cpp)
ament_target_dependencies(hello_cpp rclcpp)

add_executable(apriltag_node src/apriltag_node.cpp)
ament_target_dependencies(apriltag_node rclcpp OpenCV)
target_link_libraries(apriltag_node apriltag ${OpenCV_LIBS})

add_executable(beboptag src/beboptag.cpp)
ament_target_dependencies(beboptag
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)
target_link_libraries(beboptag apriltag ${OpenCV_LIBS})

add_executable(tf_cam src/tf_cam.cpp)
ament_target_dependencies(tf_cam
  rclcpp
  tf2_ros
  geometry_msgs
)

add_executable(tf_tag_bebop src/tf_tag_bebop.cpp)
ament_target_dependencies(tf_tag_bebop
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  tf2_ros
  geometry_msgs
)
target_link_libraries(tf_tag_bebop apriltag ${OpenCV_LIBS})

# ---------------- Install ----------------
install(TARGETS
  hello_cpp
  apriltag_node
  beboptag
  tf_cam
  tf_tag_bebop
  DESTINATION lib/${PROJECT_NAME}
)

# Python nodes
install(PROGRAMS
  nero_drone/hello_py.py
  nero_drone/apriltag.py
  nero_drone/takeoff.py
  nero_drone/land.py
  nero_drone/demo_takeoff_land.py

  DESTINATION lib/${PROJECT_NAME}
)

# Launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# URDF + meshes + others
install(DIRECTORY
  urdf
  meshes
  others
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

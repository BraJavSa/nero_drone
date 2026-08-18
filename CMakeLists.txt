cmake_minimum_required(VERSION 3.5)
project(neroControl)

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
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Buscar explícitamente la librería de AprilTag para evitar "undefined reference"
find_library(APRILTAG_LIB NAMES apriltag REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(joy_to_cmd src/joy_to_cmd.cpp)
ament_target_dependencies(joy_to_cmd rclcpp sensor_msgs geometry_msgs std_msgs)
target_link_libraries(joy_to_cmd ${OpenCV_LIBS})

add_executable(april_tag_node src/april_tag_node.cpp)
ament_target_dependencies(april_tag_node rclcpp OpenCV)
target_link_libraries(april_tag_node apriltag ${OpenCV_LIBS})

add_executable(bebop_tag src/bebop_tag.cpp)
ament_target_dependencies(bebop_tag rclcpp sensor_msgs cv_bridge OpenCV)
target_link_libraries(bebop_tag apriltag ${OpenCV_LIBS})

add_executable(tf_camera src/tf_camera.cpp)
ament_target_dependencies(tf_camera rclcpp tf2 tf2_ros geometry_msgs nav_msgs std_msgs tf2_geometry_msgs)

add_executable(tf_tag_bebop src/tf_tag_bebop.cpp)
ament_target_dependencies(tf_tag_bebop rclcpp sensor_msgs cv_bridge OpenCV tf2_ros geometry_msgs visualization_msgs tf2 tf2_geometry_msgs)
target_link_libraries(tf_tag_bebop apriltag ${OpenCV_LIBS})

add_executable(velocity_logger src/velocity_logger.cpp)
ament_target_dependencies(velocity_logger rclcpp nav_msgs geometry_msgs std_msgs tf2 tf2_geometry_msgs)

add_executable(safety_watchdog src/safe_watchdog.cpp)
ament_target_dependencies(safety_watchdog rclcpp geometry_msgs std_msgs)

add_executable(safe_bebop_republisher src/bridge_cmd.cpp)
ament_target_dependencies(safe_bebop_republisher rclcpp geometry_msgs)

add_executable(isfly src/isfly.cpp)
ament_target_dependencies(isfly rclcpp std_msgs)

add_executable(tf_odom_to_map src/tf_odom_map.cpp)
ament_target_dependencies(tf_odom_to_map rclcpp tf2_ros geometry_msgs)

add_executable(gt_mocap_odom_cpp src/gt_mocap_odom.cpp)
ament_target_dependencies(gt_mocap_odom_cpp rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(gt_velocity_logger src/gt_velocity_logger.cpp)
ament_target_dependencies(gt_velocity_logger rclcpp geometry_msgs nav_msgs tf2)

add_executable(gt_tf_camera src/gt_tf_camera.cpp)
ament_target_dependencies(gt_tf_camera rclcpp geometry_msgs std_msgs tf2 tf2_ros)


# ---------------- Install ----------------
install(TARGETS
  joy_to_cmd april_tag_node bebop_tag tf_camera tf_tag_bebop velocity_logger
  safety_watchdog safe_bebop_republisher isfly 
  tf_odom_to_map gt_mocap_odom_cpp gt_velocity_logger gt_tf_camera
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------- Python nodes, Launch, Config, URDF ----------------
install(PROGRAMS
  nero_control/offline_plotter.py
  nero_control/apriltag.py
  nero_control/bebop_control_gui.py
  nero_control/position_ref.py
  nero_control/extended_controller.py
  nero_control/sensor_fusion.py
  nero_control/initial_frame.py
  nero_control/realtime_graphical.py
  nero_control/body_realtime_velocity.py
  nero_control/velocity_gains_optimizer.py
  nero_control/trajectory_ref.py
  nero_control/multi_trajectory_ref.py
  nero_control/ibvs_controller.py
  nero_control/article_plotter.py
  nero_control/trajectory_graphical_3d.py
  nero_control/vel_controller.py
  nero_control/pioneer_joy.py
  nero_control/velocity_test.py
  nero_control/land_cmd.py
  nero_control/body_velocities_plotter.py
  nero_control/model_logger.py
  nero_control/gt/gt_mocap_odom.py
  nero_control/gt/gt_extended_controller.py
  nero_control/gt/gt_body_realtime_velocity.py
  nero_control/gt/gt_ibvs_controller.py
  nero_control/gt/gt_vel_controller.py
  nero_control/gt/gt_velocity_test.py
  nero_control/gt/gt_realtime_graphical.py
  nero_control/gt/gt_model_logger.py
  nero_control/gt/gt_position_ref.py
  nero_control/gt/gt_trajectory_ref.py
  nero_control/gt/gt_gains_optimizer.py
  nero_control/gt/gt_rk4_simulation.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf meshes others DESTINATION share/${PROJECT_NAME})

ament_package()

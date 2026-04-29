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

# ... (Tus otros ejecutables: Joy2Cmd, apriltag_node, beboptag, ibvs se mantienen igual)

add_executable(Joy2Cmd src/Joy2Cmd.cpp)
ament_target_dependencies(Joy2Cmd rclcpp sensor_msgs geometry_msgs std_msgs)
target_link_libraries(Joy2Cmd ${OpenCV_LIBS})

add_executable(apriltag_node src/apriltag_node.cpp)
ament_target_dependencies(apriltag_node rclcpp OpenCV)
target_link_libraries(apriltag_node apriltag ${OpenCV_LIBS})

add_executable(beboptag src/beboptag.cpp)
ament_target_dependencies(beboptag rclcpp sensor_msgs cv_bridge OpenCV)
target_link_libraries(beboptag apriltag ${OpenCV_LIBS})

add_executable(ibvs src/ibvs.cpp)
ament_target_dependencies(ibvs rclcpp sensor_msgs cv_bridge OpenCV )
target_link_libraries(ibvs apriltag ${OpenCV_LIBS})


add_executable(tf_cam src/tf_cam.cpp)
ament_target_dependencies(tf_cam rclcpp tf2 tf2_ros geometry_msgs nav_msgs std_msgs tf2_geometry_msgs)

add_executable(tf_tag_bebop src/tf_tag_bebop.cpp)
ament_target_dependencies(tf_tag_bebop rclcpp sensor_msgs cv_bridge OpenCV tf2_ros geometry_msgs visualization_msgs tf2 tf2_geometry_msgs)
target_link_libraries(tf_tag_bebop apriltag ${OpenCV_LIBS})

add_executable(velocity_logger src/velocity_logger.cpp)
ament_target_dependencies(velocity_logger rclcpp nav_msgs geometry_msgs std_msgs tf2 tf2_geometry_msgs)

add_executable(ref_vec_filter src/filtred.cpp)
ament_target_dependencies(ref_vec_filter rclcpp geometry_msgs std_msgs visualization_msgs tf2_geometry_msgs)

add_executable(safety_watchdog src/safe_watchdog.cpp)
ament_target_dependencies(safety_watchdog rclcpp geometry_msgs std_msgs)

add_executable(tag_pos src/tag_pos.cpp)
ament_target_dependencies(tag_pos rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs cv_bridge OpenCV tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(tag_pos apriltag ${OpenCV_LIBS})

add_executable(safe_bebop_republisher src/bridge_cmd.cpp)
ament_target_dependencies(safe_bebop_republisher rclcpp geometry_msgs)

add_executable(isfly src/isfly.cpp)
ament_target_dependencies(isfly rclcpp std_msgs)

add_executable(tf_odom_to_map src/tf_odom_map.cpp)
ament_target_dependencies(tf_odom_to_map rclcpp tf2_ros geometry_msgs)

# --- EJECUTABLE PBVS ACTUALIZADO ---
add_executable(pbvs src/pbvs.cpp)
ament_target_dependencies(pbvs
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)
# Enlace explícito a la librería encontrada y OpenCV
target_link_libraries(pbvs ${APRILTAG_LIB} ${OpenCV_LIBS})

# ---------------- Install ----------------
install(TARGETS
  Joy2Cmd apriltag_node beboptag tf_cam tf_tag_bebop velocity_logger
  safety_watchdog safe_bebop_republisher ref_vec_filter isfly ibvs
  tag_pos tf_odom_to_map pbvs 
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------- Python nodes, Launch, Config, URDF ----------------
install(PROGRAMS
  neroControl/graphical.py
  neroControl/apriltag.py
  neroControl/bebop_control_gui.py
  neroControl/ref_pos.py
  neroControl/controlador.py
  neroControl/sim.py
  neroControl/sim120.py
  neroControl/altitude.py
  neroControl/altitude_sim.py
  neroControl/initial_frame.py
  neroControl/ontime_graphical.py
  neroControl/ref_trayectory.py
  neroControl/ref_multitrayectoria.py
  neroControl/pypbsv.py
  neroControl/pyibsv.py
  neroControl/article_plotter.py
  neroControl/trajectory_graphical_3d.py
  neroControl/vel_controller.py
  neroControl/pioneer_joy.py
  neroControl/prueba_velocidad.py
  neroControl/land.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf meshes others DESTINATION share/${PROJECT_NAME})

ament_package()

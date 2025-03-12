cmake_minimum_required(VERSION 3.0.2)
project(hl2ss_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  camera_info_manager
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../viewer
)

catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/../viewer
)

add_executable(camera_info_publisher src/camera_info_publisher.cpp)
target_link_libraries(camera_info_publisher
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/client_stream_rm_vlc.py
  scripts/client_stream_si.py
  scripts/pose_to_path.py
  scripts/client_stream_si_imu.py
  scripts/si_to_odom_pub.py
  scripts/sample_uv_to_3d_vlc.py
  scripts/cacti_dataset_stream_writer.py
  scripts/cacti_test.py
  scripts/tf_republisher.py
  scripts/tf_static_publishers.py
  scripts/camera_info_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

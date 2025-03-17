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
  scripts/hololens_vlc_pub.py
  scripts/hololens_vlc_multi_sync_pub.py
  scripts/vlc_uv_to_3d.py
  scripts/hololens_pose_pub.py
  scripts/pose_to_path.py
  scripts/hololens_imu_pub.py
  scripts/hololens_odom_pub.py
  scripts/cacti_dataset_stream_writer.py
  scripts/tf_republisher.py
  scripts/hololens_static_tf_pub.py
  scripts/cal_eye_in_hand_data_collect.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

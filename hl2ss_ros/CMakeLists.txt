cmake_minimum_required(VERSION 3.0.2)
project(hl2ss_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
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

catkin_install_python(PROGRAMS
  scripts/client_stream_rm_vlc.py
  scripts/client_stream_si.py
  scripts/pose_to_path.py
  scripts/client_stream_si_imu.py
  scripts/si_to_odom_pub.py
  scripts/sample_uv_to_3d_vlc.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

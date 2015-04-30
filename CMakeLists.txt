cmake_minimum_required(VERSION 2.8.3)
project(knowrob_semantic_map_tools)

find_package(
  catkin
  REQUIRED COMPONENTS
    rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    geometry_msgs
    rosprolog
    json_prolog
    json_prolog_msgs
    knowrob_actions
    knowrob_common
    knowrob_map_tools
    knowrob_objects
    knowrob_semantic_map_msgs
    knowrob_semantic_map_to_owl
    knowrob_vis
    tf_conversions
    visualization_msgs
)

install(
  PROGRAMS
    bin/semantic_map.py
    bin/semantic_map_test.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bin
)

install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(
  DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog
)
install(
  DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

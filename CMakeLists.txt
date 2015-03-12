cmake_minimum_required(VERSION 2.8.3)
project(knowrob_semantic_map_tools)

find_package(
  catkin
)

catkin_package(
  DEPENDS
    rosprolog
    json_prolog
    knowrob_common
    knowrob_semantic_map_to_owl
)

install(
  DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog
)

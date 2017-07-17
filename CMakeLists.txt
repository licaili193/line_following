cmake_minimum_required(VERSION 2.8.3)
project(line_following)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
)

add_service_files(FILES LineFollowing.srv)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}
)

add_executable(lf_cmd src/LF_cmd.cpp)

target_link_libraries(lf_cmd
   ${catkin_LIBRARIES}
)

add_executable(lf_test 
src/LF_test.cpp
src/maps/Track_base.cpp
src/maps/Line_track.cpp
src/maps/Arc_track.cpp
src/maps/Map_builder.cpp)

target_link_libraries(lf_test
   ${catkin_LIBRARIES}
)

add_dependencies(lf_test ${PROJECT_NAME}_generate_messages_cpp)

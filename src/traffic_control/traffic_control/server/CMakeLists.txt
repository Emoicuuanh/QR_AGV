cmake_minimum_required(VERSION 3.0.2)
project(server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  nav_msgs
  tuw_multi_robot_msgs
)

find_package(jsoncpp CONFIG REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(server_traffic_control src/server_traffic_control.cpp)
add_executable(client_agv src/client_agv.cpp)
target_link_libraries(server_traffic_control
  ${catkin_LIBRARIES}
  jsoncpp_lib
)
target_link_libraries(client_agv
  ${catkin_LIBRARIES}
  jsoncpp_lib
)
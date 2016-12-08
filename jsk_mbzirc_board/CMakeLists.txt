cmake_minimum_required(VERSION 2.8.3)
project(jsk_mbzirc_board)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy message_generation)

add_service_files(
  FILES
  Magnet.srv)

generate_messages(
  DEPENDENCIES)

catkin_package(
  CATKIN_DEPENDS
)


#uav_teleop_keyboard node



#goblin_gopose node
#add_executable(serial_board src/serial_board/serial_board.cpp)
#target_link_libraries(serial_board ${catkin_LIBRARIES})





cmake_minimum_required(VERSION 2.8.3)
project(jsk_task3_m100)
set(CMAKE_BUILD_TYPE Debug)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp rospy cv_bridge image_transport tf message_filters
nav_msgs sensor_msgs dynamic_reconfigure jsk_mbzirc_board dji_sdk dji_sdk_lib tf)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
find_package(OpenCV REQUIRED)
add_message_files(
FILES
ProjectionMatrix.msg
)
generate_messages(
DEPENDENCIES
sensor_msgs
std_msgs)
#add gpu support
set(ENABLE_CUDA ON)
find_package(OpenMP)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
set(EIGEN_INCLUDE_DIRS "/usr/include/eigen3/")
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
if(ENABLE_CUDA)
find_package(CUDA REQUIRED)
set(CUDA_INCLUDE_DIRS "/usr/local/cuda/")
include_directories(${CUDA_INCLUDE_DIRS})
#add gpu support for uav_img2pointcloud node
add_definitions(-DGPU_EN)
#cuda_add_library(UAV_IMG2POINTCLOUD_CU_LIB ${LIB_TYPE} src/img2pointcloud/uav_img2pointcloud.cu)
#add_executable(uav_img2pointcloud src/img2pointcloud/uav_img2pointcloud.cpp)
#add_dependencies(uav_img2pointcloud UAV_IMG2POINTCLOUD_CU_LIB)
#target_link_libraries(uav_img2pointcloud ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CUDA_LIBRARIES} UAV_IMG2POINTCLOUD_CU_LIB)

#task3 vision node
add_executable(task3_vision src/task3vision/task3vision.cpp)
target_link_libraries(task3_vision ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CUDA_LIBRARIES})
else()
#add_executable(uav_img2pointcloud src/img2pointcloud/uav_img2pointcloud.cpp)
#target_link_libraries(uav_img2pointcloud ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#task3 vision node
add_executable(task3_vision src/task3vision/task3vision.cpp)
target_link_libraries(task3_vision ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

endif(ENABLE_CUDA)
#uav_teleop_keyboard node
add_executable(uav_teleop_keyboard src/teleop/uav_teleop_keyboard.cpp)
target_link_libraries(uav_teleop_keyboard ${catkin_LIBRARIES})
#uav_teleop_joy node
add_executable(uav_teleop_joy src/teleop/uav_teleop_joy.cpp)
target_link_libraries(uav_teleop_joy ${catkin_LIBRARIES})
#gimbal_control node
add_executable(gimbal_control src/gimbalcontrol/gimbal_control.cpp)
target_link_libraries(gimbal_control ${catkin_LIBRARIES})
#uav_treasure_pick node
add_executable(uav_treasure_pick src/treasurepick/uav_treasure_pick.cpp)
target_link_libraries(uav_treasure_pick ${catkin_LIBRARIES})
#uav_treasure_move node
#add_executable(uav_treasure_move src/treasurepick/uav_treasure_move.cpp)
#target_link_libraries(uav_treasure_move ${catkin_LIBRARIES})
######################
## CFG Generation ##
######################
generate_dynamic_reconfigure_options(
cfg/Filterparam.cfg
#...
)
# make sure configure headers are built before any node using them
add_dependencies(task3_vision ${PROJECT_NAME}_gencfg)
###################################
## catkin specific configuration ##
###################################
catkin_package(
CATKIN_DEPENDS
)
#############
## Install ##
#############
install(DIRECTORY launch urdf config
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

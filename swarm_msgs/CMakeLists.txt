cmake_minimum_required(VERSION 2.8.3)
project(swarm_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(FILES
   FormationParam.msg
   FieldParam.msg
   CommonParams.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   FormationSrv.srv
   FloatSrv.srv
   FieldSrv.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
 CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)


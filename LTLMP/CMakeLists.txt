cmake_minimum_required(VERSION 2.8.3)
project(rtltl)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
   FILES
  #  activity.msg
   confirmation.msg
   pose.msg
   knowledge.msg
   activity_pose.msg
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
  #  activity.msg
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ltl3
 CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


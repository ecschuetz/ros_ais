cmake_minimum_required(VERSION 3.0.2)
project(ros_ais)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
)
 add_message_files(
   FILES
   ros_AIS123.msg
   ros_AIS4_11.msg
   ros_AIS8_dac200.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_ais
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib

CATKIN_DEPENDS message_runtime

)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

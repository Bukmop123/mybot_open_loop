cmake_minimum_required(VERSION 2.8.3)
project(mybot_msg)

find_package(catkin REQUIRED COMPONENTS
	std_msgs
	message_generation
)


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   msgMybot_basicMovement.msg
   msgMybot_detailMovement.msg
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs 
 )


catkin_package(
  CATKIN_DEPENDS 
	std_msgs
   	message_runtime
)



include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)






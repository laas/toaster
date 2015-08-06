cmake_minimum_required(VERSION 2.8.3)
project(toaster_visualizer)

find_package(catkin REQUIRED COMPONENTS roscpp rospy genmsg toaster_msgs cmake_modules roslib)
find_package(cmake_modules REQUIRED)
find_package(TinyXML REQUIRED)
 
catkin_package(
  CATKIN_DEPENDS toaster_msgs roslib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})


add_executable(run src/run.cpp)
target_link_libraries(run ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})
add_dependencies(run beginner_tutorials_generate_messages_cpp)

 
 add_executable(toaster_com src/toaster_com.cpp)
target_link_libraries(toaster_com ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})
add_dependencies(toaster_com beginner_tutorials_generate_messages_cpp)
 
 
 

catkin_package(

)


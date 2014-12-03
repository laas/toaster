cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
set(BOOST_ROOT /home/gmilliez/devel/boost_1_56_0/)
set(Boost_NO_SYSTEM_PATHS ON)

find_package(Boost REQUIRED COMPONENTS)# system)
include_directories(${Boost_INCLUDE_DIRS} /home/gmilliez/devel/include)
rosbuild_add_executable(pdg src/MorseHumanReader.cpp src/HumanReader.cpp src/Pr2RobotReader.cpp src/RobotReader.cpp src/ObjectReader.cpp src/main.cpp)
target_link_libraries(pdg /home/gmilliez/devel/lib/libtoaster.so)


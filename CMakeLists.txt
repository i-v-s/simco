cmake_minimum_required(VERSION 2.8.3)
project(simco)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	sensor_msgs
	geometry_msgs
	svo_msgs
        rosbag
)

#find_package(Eigen)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES mavros
  CATKIN_DEPENDS roscpp geometry_msgs svo_msgs sensor_msgs rosbag
  DEPENDS Boost
)

include_directories(
	${catkin_INCLUDE_DIRS}
	#${Eigen_INCLUDE_DIRS}
)

add_executable(simco simco.cpp integrator.cpp integrator.h copter.cpp)
#add_dependencies(flyer mavros_msgs)
#add_dependencies(flyer mavros_msgs_generate_messages_cpp)
#add_dependencies(flyer geometry_msgs_generate_messages_cpp)

target_link_libraries(simco
  ${catkin_LIBRARIES}
  gtest #gtest_main gtest pthread
)

add_executable(simco_test simco_test.cpp integrator.cpp trapint.cpp)

target_link_libraries(simco_test
  ${catkin_LIBRARIES}
  gtest #gtest_main gtest pthread
)

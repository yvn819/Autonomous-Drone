cmake_minimum_required(VERSION 2.8.3)
project(filter)


include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wl,--no-as-needed")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

# add_compile_options(-pthread)

#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/home/li/catkin_ws/devel_isolated/tf2_geometry_msgs")
#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/home/li/catkin_ws/devel_isolated/tf2")
#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/home/li/catkin_ws/devel_isolated/tf2_ros")
#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/home/li/catkin_ws/devel_isolated/tf2_msgs")
find_package(PCL REQUIRED COMPONENTS common segmentation)

find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
sensor_msgs
tf2
geometry_msgs
tf2_ros
tf2_geometry_msgs
nav_msgs
octomap_msgs
octomap_ros
)
# find_package(Threads REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs pcl_ros tf2 
)



include_directories(${catkin_INCLUDE_DIRS}
${tf2_INCLUDE_DIRS} 
${geometry_msgs_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(filter_node src/filter_node.cpp)
target_link_libraries(filter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${TF2_LIBRARIES} Threads::Threads ${geometry_msgs_LIBRARIES})
# set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/lib/x86_64-linux-gnu")


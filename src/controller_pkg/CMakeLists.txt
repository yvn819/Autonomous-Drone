cmake_minimum_required(VERSION 3.0.2)
project(controller_pkg)

include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  geometry_msgs 
  mav_msgs
  eigen_conversions
  tf_conversions
  nav_msgs
)

catkin_package(CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  mav_msgs 
  tf
  eigen_conversions
  tf_conversions
  nav_msgs
)

find_package(Eigen3)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(controller_node src/controller_node.cpp)
add_dependencies(controller_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(controller_node ${catkin_LIBRARIES})

add_executable(traj_publisher src/traj_publisher.cpp)
add_dependencies(traj_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(traj_publisher ${catkin_LIBRARIES})

add_executable(rotate_in_place src/rotate_in_place.cpp)
add_dependencies(rotate_in_place ${catkin_EXPORTED_TARGETS})
target_link_libraries(rotate_in_place ${catkin_LIBRARIES})

add_executable(move2cave src/move2cave.cpp)
add_dependencies(move2cave ${catkin_EXPORTED_TARGETS})
target_link_libraries(move2cave ${catkin_LIBRARIES})

add_executable(turn src/turn.cpp)
add_dependencies(turn ${catkin_EXPORTED_TARGETS})
target_link_libraries(turn ${catkin_LIBRARIES})

add_executable(explore src/explore.cpp)
add_dependencies(explore ${catkin_EXPORTED_TARGETS})
target_link_libraries(explore ${catkin_LIBRARIES})

add_executable(state_machine src/state_machine.cpp)
add_dependencies(state_machine ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_machine ${catkin_LIBRARIES})



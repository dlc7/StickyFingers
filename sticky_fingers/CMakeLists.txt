cmake_minimum_required(VERSION 2.8.3)
project(sticky_fingers)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

find_package(catkin_simple REQUIRED)
# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)
catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Executable
cs_add_executable(finger_control_dummy_node src/finger_control_dummy_node.cpp)

#Library
add_library(sticky_fingers src/sticky_fingers.cpp)
target_link_libraries(sticky_fingers ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)

cs_install()
cs_export()

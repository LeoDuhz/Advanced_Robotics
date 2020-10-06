cmake_minimum_required(VERSION 2.8.3)
project(course_agv_icp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  tf
  tf_conversions
)
find_package(Eigen3 REQUIRED)
#find_package(PCL 1.8 REQUIRED)
find_package( PCL REQUIRED COMPONENT  common io filters)
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization)

add_definitions(-std=c++11 -O2 -g -DNDEBUG)

catkin_package (
#	CATKIN_DEPENDS roscpp
)


include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(icp src/icp.cpp)
target_link_libraries(icp
${catkin_LIBRARIES}
${PCL_LIBRARIES}
)

add_executable(tf src/tf.cpp)
target_link_libraries(tf
${catkin_LIBRARIES}
${PCL_LIBRARIES}
)

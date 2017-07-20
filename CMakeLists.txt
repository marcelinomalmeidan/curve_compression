cmake_minimum_required(VERSION 2.8.3)
project(curve_compression)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)

add_service_files( FILES
  compressCurve.srv
)

generate_messages( DEPENDENCIES
  nav_msgs
  std_msgs
)

catkin_package(
  DEPENDS Eigen
)

###########
## Build ##
###########
 
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

#Service node
add_executable(curveCompression src/curveCompression.cpp
                                src/geometryFunctions.cpp)
target_link_libraries(curveCompression ${catkin_LIBRARIES})
 
#This makes sure that messages and services are compiled before the rest
add_dependencies(curveCompression curveCompression_generate_messages_cpp)
add_dependencies(curveCompression ${catkin_EXPORTED_TARGETS})

#Example client node
add_executable(curveCompressionClient src/curveCompressionClient.cpp)
target_link_libraries(curveCompressionClient ${catkin_LIBRARIES})
add_dependencies(curveCompressionClient curveCompressionClient_generate_messages_cpp)
add_dependencies(curveCompressionClient ${catkin_EXPORTED_TARGETS})

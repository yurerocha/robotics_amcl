cmake_minimum_required(VERSION 3.0.2)
project(robotics_amcl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -DWITHOUT_NUMPY -I/usr/local/include/python3.8 -lpython3.8)

list(APPEND MCL_PACKAGES
  roscpp
  nodelet
  std_msgs
  geometry_msgs
  sensor_msgs
  map_server
  visualization_msgs
  tf
)

find_package(catkin REQUIRED COMPONENTS 
  ${MCL_PACKAGES}
)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

catkin_package(
  CATKIN_DEPENDS ${MCL_PACKAGES}
)

###########
## Build ##
###########

include_directories(
  include 
  /opt/ros/noetic/include
  # /usr/local/include/opencv4
  # ../cvplot/CvPlot/inc
  # /home/yure/catkin_ws/src/ekf_mcl/cvplot/CvPlot/inc
  /home/yure/catkin_ws/src/ekf_mcl/matplotlib
  /usr/include/python3.8
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(MCL_SRC_FILES
  # src/Utils.cpp
  # src/EKF.cpp
  src/Data.cpp
  src/MCL.cpp
)

add_executable(${PROJECT_NAME} src/main.cpp ${MCL_SRC_FILES})

# target_compile_definitions(${PROJECT_NAME} PUBLIC CVPLOT_HEADER_ONLY)

# link_directories(/opt/ros/noetic/lib/)
target_link_libraries(${PROJECT_NAME} 
  /opt/ros/noetic/lib/librosconsole.so 
  /opt/ros/noetic/lib/libroscpp.so
  /opt/ros/noetic/lib/libroscpp_serialization.so
  /opt/ros/noetic/lib/libtf.so
  /opt/ros/noetic/lib/librostime.so
  Threads::Threads
  ${roscpp_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS}
)

#############
## Install ##
#############

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

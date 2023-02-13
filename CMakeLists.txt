cmake_minimum_required(VERSION 3.0.2)
project(realsense_interfaces)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  pcl_ros
  tf2
  tf2_geometry_msgs
  tf2_ros
  cv_bridge
  image_transport
)

## System dependencies are found with CMake's conventions
find_package(realsense2 REQUIRED) #realsense2_FOUND will be set to indicate whether the package was found.
find_package(OpenCV REQUIRED)
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES realsense_interfaces
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}_D435Interface src/D435Interface.cpp)

target_link_libraries(${PROJECT_NAME}_D435Interface
    ${catkin_LIBRARIES}
    ${realsense2_LIBRARY}
    ${OpenCV_LIBS}
)

add_library(${PROJECT_NAME}_D435Collector src/D435Collector.cpp)

target_link_libraries(${PROJECT_NAME}_D435Collector
    ${catkin_LIBRARIES}
    ${realsense2_LIBRARY}
    ${PROJECT_NAME}_D435Interface
)

# add_executable(${PROJECT_NAME}_D435Sdk src/D435Sdk.cpp)
# 
# target_link_libraries(${PROJECT_NAME}_D435Sdk
#     ${catkin_LIBRARIES}
#     ${realsense2_LIBRARY}
#     ${PROJECT_NAME}_D435Interface
# 
# )
# set_target_properties(${PROJECT_NAME}_D435Sdk PROPERTIES OUTPUT_NAME "D435Sdk")

add_executable(${PROJECT_NAME}_main src/main.cpp)

target_link_libraries(${PROJECT_NAME}_main
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_D435Collector

)
set_target_properties(${PROJECT_NAME}_main PROPERTIES OUTPUT_NAME "main")
    
    

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tpo_embedded.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

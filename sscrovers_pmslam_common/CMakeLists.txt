cmake_minimum_required(VERSION 2.8.3)
project(sscrovers_pmslam_common)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS angles cv_bridge geometry_msgs image_transport nav_msgs roscpp sensor_msgs std_msgs tf message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   ControlVector.msg
   DynamicArray.msg
   DynamicArrayFeature.msg
   IDPoint.msg
   Landmark.msg
   Map3D.msg
   MapOut_old.msg
   PairedPoints3D.msg
   PMSlamData.msg
   Points3D.msg
   PtPairs.msg
   PtPairs.msg
   Trajectory.msg
   Trajectory_old.msg
   SPoint.msg
   SALVector.msg
   RoverState.msg
   featureMap.msg
   extraFeature.msg
   extraFeatures.msg
   pairs.msg
   unadjustedPairs.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs   nav_msgs   sensor_msgs   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES sscrovers_pmslam_common
  CATKIN_DEPENDS angles cv_bridge geometry_msgs image_transport nav_msgs roscpp sensor_msgs std_msgs tf message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
 add_library(sscrovers_pmslam_common
   src/${PROJECT_NAME}/afeature.cpp
   src/${PROJECT_NAME}/feature.cpp
   src/${PROJECT_NAME}/feature_vector.cpp
   src/${PROJECT_NAME}/rover_state.cpp
 )

## Declare a cpp executable
# add_executable(sscrovers_pmslam_common_node src/sscrovers_pmslam_common_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(sscrovers_pmslam_common_node sscrovers_pmslam_common_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(sscrovers_pmslam_common_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS sscrovers_pmslam_common sscrovers_pmslam_common_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sscrovers_pmslam_common.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

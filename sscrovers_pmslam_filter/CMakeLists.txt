cmake_minimum_required(VERSION 2.8.3)
project(sscrovers_pmslam_filter)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport nav_msgs roscpp sscrovers_pmslam_common tf)

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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   nav_msgs
# )

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
#  LIBRARIES sscrovers_pmslam_filter
   CATKIN_DEPENDS cv_bridge image_transport nav_msgs roscpp sscrovers_pmslam_common tf
#  DEPENDS system_lib
)

###########
## Build ##
###########


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}  ${catkin_INCLUDE_DIRS}/FindOpenCV.cmake)
find_package(OpenCV 2)
include_directories(${CMAKE_SOURCE_DIR}/include
                                      ${OpenCV_INCLUDE_DIRS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(sscrovers_pmslam_filter
#   src/${PROJECT_NAME}/sscrovers_pmslam_filter.cpp
# )

set (CMAKE_CXX_FLAGS "-std=c++0x")

## Declare a cpp executable
 add_executable(ftr_node
	src/${PROJECT_NAME}/ftr_core.cpp
	src/${PROJECT_NAME}/information_filter_ftr.cpp
	src/${PROJECT_NAME}/EIF.cpp
			src/${PROJECT_NAME}/RoverState.cpp
			src/${PROJECT_NAME}/Landmark.cpp
			src/${PROJECT_NAME}/Map.cpp
	src/${PROJECT_NAME}/features_db.cpp
	src/${PROJECT_NAME}/SURFPoint.cpp
 )



## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(sscrovers_pmslam_filter_node sscrovers_pmslam_filter_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(ftr_node
   ${catkin_LIBRARIES}
 )


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
# install(TARGETS sscrovers_pmslam_filter sscrovers_pmslam_filter_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sscrovers_pmslam_filter.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

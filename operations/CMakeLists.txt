cmake_minimum_required(VERSION 3.0.2)
project(operations)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib_msgs
  genmsg
  message_generation
  actionlib
  std_msgs
  utils
  geometry_msgs
  perception
  planning
  gazebo_msgs
  srcp2_msgs
  tf2
  tf2_ros
  maploc
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   #TrajectoryWithVelocities.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Spiral.srv
)

## Generate actions in the 'action' folder
# add_action_files(DIRECTORY action FILES DoDishes.action)
add_action_files(DIRECTORY action
  FILES
  Navigation.action
  NavigationVision.action
  ParkRobot.action
  NavigationVision.action
  ResourceLocaliser.action
  Hauler.action
  Excavator.action
  SolarMode.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
  srcp2_msgs
)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
 LIBRARIES operations
 CATKIN_DEPENDS roscpp rospy actionlib_msgs std_msgs message_runtime utils geometry_msgs srcp2_msgs perception maploc planning
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
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/navigation/navigation_algorithm.cpp
  src/navigation/navigation_server.cpp
  src/navigation/solar_charging_server.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/scout/operations_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#######################################################################################################

# Drives both robots forwards in task 2. Example node

add_executable(start_nav_server src/navigation/start_nav_server.cpp)
add_dependencies(start_nav_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(start_nav_server ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(wheel_speed_processing src/navigation/wheel_speed_processing.cpp)
add_dependencies(wheel_speed_processing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheel_speed_processing
  ${catkin_LIBRARIES}
)

add_executable(navigation_vision_server src/navigation/navigation_vision_server.cpp)
add_dependencies(navigation_vision_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigation_vision_server ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(start_solar_charging_server src/navigation/start_solar_charging_server.cpp)
add_dependencies(start_solar_charging_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(start_solar_charging_server ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

###################
#### EXCAVATOR ####
###################

add_executable(excavator_actionlib_server src/excavator/excavator_server.cpp)
add_dependencies(excavator_actionlib_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(excavator_actionlib_server ${PROJECT_NAME}
${catkin_LIBRARIES}
)


add_executable(excavator_pid_node src/excavator/excavator_pid.cpp)
add_dependencies(excavator_pid_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(excavator_pid_node ${PROJECT_NAME}
${catkin_LIBRARIES}
)

##################
##### HAULER #####
##################

add_executable(hauler_actionlib_server src/hauler/hauler_server.cpp)
add_dependencies(hauler_actionlib_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hauler_actionlib_server ${PROJECT_NAME}
${catkin_LIBRARIES}
)

add_executable(park_hauler_server src/hauler/park_hauler_server.cpp)
add_dependencies(park_hauler_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(park_hauler_server ${PROJECT_NAME}
${catkin_LIBRARIES}
)


###################
###### SCOUT ######
###################

add_executable(resource_localiser src/scout/resource_localiser.cpp)
add_dependencies(resource_localiser ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(resource_localiser
${catkin_LIBRARIES}
)

add_executable(resource_localiser_tester src/scout/resource_localiser_tester.cpp)
add_dependencies(resource_localiser_tester ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(resource_localiser_tester
  ${catkin_LIBRARIES}
)

add_executable(scout_search src/scout/scout_search.cpp)
add_dependencies(scout_search ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scout_search ${PROJECT_NAME}
${catkin_LIBRARIES}
)

add_executable(spiral_points_publisher src/clients/spiral_points_publisher.cpp)
add_dependencies(spiral_points_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(spiral_points_publisher ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#########################
###### ALL CLIENTS ######
#########################

# Drives both robots forwards in task 2. Example node
add_executable(navigation_client src/clients/navigation_client.cpp)
add_dependencies(navigation_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigation_client
  ${catkin_LIBRARIES}
)

add_executable(navigation_vision_client src/clients/navigation_vision_client.cpp)
add_dependencies(navigation_vision_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigation_vision_client
  ${catkin_LIBRARIES}
)

add_executable(excavator_actionlib_client src/clients/excavator_client.cpp)
add_dependencies(excavator_actionlib_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(excavator_actionlib_client ${PROJECT_NAME}
${catkin_LIBRARIES}
)

add_executable(hauler_actionlib_client src/clients/hauler_client.cpp)
add_dependencies(hauler_actionlib_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hauler_actionlib_client ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(park_hauler_client src/clients/park_hauler_client.cpp)
add_dependencies(park_hauler_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(park_hauler_client ${PROJECT_NAME}
${catkin_LIBRARIES}
)

add_executable(solar_charging_client src/clients/solar_charging_client.cpp)
add_dependencies(solar_charging_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(solar_charging_client ${PROJECT_NAME}
${catkin_LIBRARIES}
)
# target_link_libraries(start_scout_sm navigation_algorithm)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination

# All the cpp executable nodes should be listed here as well
install(TARGETS start_nav_server wheel_speed_processing navigation_vision_server 
                excavator_actionlib_server excavator_pid_node hauler_actionlib_server 
                park_hauler_server resource_localiser resource_localiser_tester 
                scout_search spiral_points_publisher navigation_client navigation_vision_client 
                excavator_actionlib_client hauler_actionlib_client park_hauler_client
                start_solar_charging_server
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_operations.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

catkin_add_gtest(${PROJECT_NAME}-test test/algorithm_tests.cpp)
target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${PROJECT_NAME})

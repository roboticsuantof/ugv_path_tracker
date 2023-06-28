cmake_minimum_required(VERSION 2.8.3)
project(ugv_path_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug) #None, Debug, Release, RelWithDebInfo, MinSizeRel

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  upo_actions
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/nav.cfg
  cfg/PathTracker.cfg
)
#generate_messages(
#  DEPENDENCIES
#)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ugv_path_tracker ${EXTERNAL_LIBS}  yaml-cpp
  CATKIN_DEPENDS roscpp upo_actions
  DEPENDS yaml-cpp
)
###########
## Build ##
###########

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)


#add_executable(test_node src/test.cpp)
#target_link_libraries(test_node ${catkin_LIBRARIES}  )
#add_dependencies(test_node ${catkin_EXPORTED_TARGETS}  ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_executable(ugv_path_tracker src/navigation.cpp)
#target_link_libraries(ugv_path_tracker ${catkin_LIBRARIES} navigator)

#add_executable(ugv_path_tracker_2 src/navigation_2.cpp)
#target_link_libraries(ugv_path_tracker_2 ${catkin_LIBRARIES} navigator)

#add_executable(nav_node src/nav_node.cpp)
#add_library(security_margin src/securityMargin.cpp)
#add_library(displacement src/displacement.cpp )
#add_dependencies(displacement ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(nav_node ${catkin_LIBRARIES} displacement security_margin )
#add_dependencies(nav_node ${PROJECT_NAME}_gencfg  ${catkin_EXPORTED_TARGETS}  ${${PROJECT_NAME}_EXPORTED_TARGETS})
#
#add_executable(sfm_nav_node src/nav_node_sfm.cpp src/securityMargin.cpp src/sfmcontroller.cpp)
#target_link_libraries(sfm_nav_node ${catkin_LIBRARIES})
#add_dependencies(sfm_nav_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS}  ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_library(security_margin src/securityMargin.cpp)
#add_library(PathTracker src/PathTracker.cpp )

#add_dependencies(PathTracker ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} )

add_executable(ugv_path_tracker_node src/ugv_path_tracker_node.cpp src/ugv_path_tracker.cpp)
target_link_libraries(ugv_path_tracker_node ${catkin_LIBRARIES})
add_dependencies(ugv_path_tracker_node ${PROJECT_NAME}_gencfg  ${catkin_EXPORTED_TARGETS}  ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(read_path_node src/read_path_node.cpp  src/read_path.cpp)
target_link_libraries(read_path_node ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(read_path_node ${PROJECT_NAME}_gencfg  ${catkin_EXPORTED_TARGETS}  ${${PROJECT_NAME}_EXPORTED_TARGETS})
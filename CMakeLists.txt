cmake_minimum_required(VERSION 2.8.3)
project(object_discovery)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(object_discovery_engine
  src/vis_wrapper.cpp 
  src/lockable.cpp
  src/utils.cpp
  src/geom_utils.cpp
  src/measures.cpp
  src/disjoint_set.cpp 
  src/segment_graph.cpp    
  src/segmenter.cpp
  src/object.cpp
  src/object_discovery_ros_interface.cpp)

# ROS node, same capabilities as segment executable below. 
cs_add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} object_discovery_engine)

# legacy executables without ROS 
cs_add_executable(browse src/browse_scenes.cpp)
target_link_libraries(browse object_discovery_engine)

cs_add_executable(segment src/segment_scene.cpp)
target_link_libraries(segment object_discovery_engine)

cs_add_executable(annotate src/annotate_bag.cpp)
target_link_libraries(annotate object_discovery_engine)

cs_add_executable(cooccurrence src/cooccurrence.cpp)
target_link_libraries(cooccurrence object_discovery_engine)

cs_install()
cs_export()

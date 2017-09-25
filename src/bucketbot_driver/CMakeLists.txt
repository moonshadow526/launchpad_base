cmake_minimum_required(VERSION 2.8.3)
project(bucketbot_driver)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    tf
    nav_msgs
    std_msgs
    sensor_msgs
    message_generation
)

add_message_files(
  FILES
  CliffEvent.msg
  Dockir.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tf nav_msgs sensor_msgs std_msgs message_runtime
)



include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(bucketbot_node src/bucketbot_node.cpp src/rotary_encoder.cpp src/ir_hasher.cpp)
target_link_libraries(bucketbot_node pigpio ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(bucketbot_node ${catkin_EXPORTED_TARGETS})

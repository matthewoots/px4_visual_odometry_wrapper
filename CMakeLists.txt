cmake_minimum_required(VERSION 3.5)
project(px4_visual)
set(CMAKE_CXX_STANDARD 14)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

add_executable(${PROJECT_NAME}_node 
   src/visual_transcription_node.cpp
   src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}_node 
   rclcpp 
   px4_msgs 
   std_msgs 
   geometry_msgs 
   tf2 
   tf2_ros 
   Eigen3)

target_include_directories(${PROJECT_NAME}_node
   PUBLIC
     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
     $<INSTALL_INTERFACE:include>)

install(TARGETS
    ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 2.8.3)
project(graph_dynamic_slam)

add_compile_options(-std=c++17 -O3 -march=native)
add_definitions(-std=c++17 -O3 -march=native)
set(CMAKE_CXX_FLAGS "-std=c++17 -O3 -march=native")


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  tf2
  tf2_ros
  sensor_msgs
  nav_msgs
  std_msgs
  tf2_geometry_msgs
  message_filters
  tf2_sensor_msgs
  ndt_omp
  visualization_msgs
)

find_package(OpenMP REQUIRED)

find_package(G2O REQUIRED)
include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(graph_dynamic_slam src/graph_dynamic_slam_node.cpp src/pointcloud_matching.cpp src/utils.h src/utils.cpp src/map_structure.h src/map_structure.cpp)


set_target_properties(graph_dynamic_slam PROPERTIES
            COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
            LINK_FLAGS ${OpenMP_CXX_FLAGS}
)

target_link_libraries(graph_dynamic_slam
  ${catkin_LIBRARIES}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SLAM3D_ADDONS}
)

install(TARGETS graph_dynamic_slam
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)




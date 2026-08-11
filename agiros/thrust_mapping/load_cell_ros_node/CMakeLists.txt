project(load_cell_ros_node)
cmake_minimum_required(VERSION 3.0.0)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(load_cell src/load_cell.cpp)
target_link_libraries(load_cell ${catkin_LIBRARIES})

cs_install()
cs_export()


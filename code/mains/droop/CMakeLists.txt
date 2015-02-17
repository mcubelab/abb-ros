cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_library(geometry_tools src/geometry_tools.cpp)
#rosbuild_add_executable(droop_testing src/test.cpp)
rosbuild_add_executable(unit_test src/unit_tests.cpp)
rosbuild_add_executable(droop_server src/droop_server.cpp)
rosbuild_add_executable(droop_client src/droop_client.cpp)
#rosbuild_add_executable(physics_toolbox src/physics_toolbox.cpp)
#target_link_libraries(droop_testing geometry_tools)

#rosbuild_add_library(polygon src/polygon.cpp)
#target_link_libraries(droop_testing polygon)
#rosbuild_add_library(polyhedron src/polyhedron.cpp)
#target_link_libraries(droop_testing polyhedron)

rosbuild_add_library(geometry src/polygon.cpp src/polyhedron.cpp)
#target_link_libraries(droop_testing geometry)
target_link_libraries(unit_test geometry)
target_link_libraries(droop_server geometry)
target_link_libraries(droop_client geometry)
#target_link_libraries(physics_toolbox geometry)

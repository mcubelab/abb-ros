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

ADD_CUSTOM_TARGET(PQPlibrary ALL DEPENDS ${PROJECT_SOURCE_DIR}/src/pqp/lib/libPQP.a)
ADD_CUSTOM_COMMAND(
  OUTPUT ${PROJECT_SOURCE_DIR}/src/pqp/lib/libPQP.a
  COMMAND make -C ${PROJECT_SOURCE_DIR}/src/pqp
  )

ADD_CUSTOM_TARGET(moveInclude ALL DEPENDS ${PROJECT_SOURCE_DIR}/include/pqp/PQP.h)
ADD_DEPENDENCIES(moveInclude PQPlibrary)
ADD_CUSTOM_COMMAND(
  OUTPUT  ${PROJECT_SOURCE_DIR}/include/pqp/PQP.h
  COMMAND cp  ${PROJECT_SOURCE_DIR}/src/pqp/include/*
              ${PROJECT_SOURCE_DIR}/include/pqp 
  DEPENDS ${PROJECT_SOURCE_DIR}/src/pqp/include/BV.h 
          ${PROJECT_SOURCE_DIR}/src/pqp/include/PQP.h
    	  ${PROJECT_SOURCE_DIR}/src/pqp/include/PQP_Compile.h
	  ${PROJECT_SOURCE_DIR}/src/pqp/include/PQP_Internal.h
	  ${PROJECT_SOURCE_DIR}/src/pqp/include/Tri.h           
  )

ADD_CUSTOM_TARGET(moveLib ALL DEPENDS ${PROJECT_SOURCE_DIR}/lib/libPQP.a)
ADD_DEPENDENCIES(moveLib PQPlibrary)
ADD_CUSTOM_COMMAND(
  OUTPUT ${PROJECT_SOURCE_DIR}/lib/libPQP.a
  COMMAND cp  ${PROJECT_SOURCE_DIR}/src/pqp/lib/libPQP.a
              ${PROJECT_SOURCE_DIR}/lib/libPQP.a
  DEPENDS ${PROJECT_SOURCE_DIR}/src/pqp/lib/libPQP.a 
  )



#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

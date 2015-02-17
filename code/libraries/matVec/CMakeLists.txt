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
#rosbuild_gensrv()

ADD_CUSTOM_COMMAND(
  OUTPUT matVec.h
  COMMAND cat ${PROJECT_SOURCE_DIR}/src/Vec.h
              ${PROJECT_SOURCE_DIR}/src/Mat.h
              ${PROJECT_SOURCE_DIR}/src/RotMat.h 
              ${PROJECT_SOURCE_DIR}/src/Quaternion.h 
              ${PROJECT_SOURCE_DIR}/src/HomogTransf.h 
              ${PROJECT_SOURCE_DIR}/src/Polynom.h 
              > ${PROJECT_SOURCE_DIR}/include/matVec/matVec.h
  DEPENDS ${PROJECT_SOURCE_DIR}/src/Vec.h 
          ${PROJECT_SOURCE_DIR}/src/Mat.h 
          ${PROJECT_SOURCE_DIR}/src/RotMat.h 
          ${PROJECT_SOURCE_DIR}/src/Quaternion.h 
          ${PROJECT_SOURCE_DIR}/src/HomogTransf.h 
          ${PROJECT_SOURCE_DIR}/src/Polynom.h
  )

ADD_CUSTOM_TARGET(makeH ALL DEPENDS matVec.h)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(matVec src/HomogTransf.cpp
                            src/Mat.cpp
                            src/Polynom.cpp
                            src/Quaternion.cpp
                            src/RotMat.cpp
                            src/Vec.cpp)

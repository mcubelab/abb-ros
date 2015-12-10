cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(ABB_PATH ${PROJECT_SOURCE_DIR}/../../ABBInterpreter)


ADD_CUSTOM_COMMAND(
  OUTPUT libABBInterpreter.a
  COMMAND +make -C ${ABB_PATH}
  DEPENDS ${ABB_PATH}/src/ABBInterpreter.cpp
  )


ADD_CUSTOM_TARGET(makeABBLIB ALL DEPENDS libABBInterpreter.a)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

include_directories(${ABB_PATH}/include)
#include_directories(src/PracticalSocket)
#include_directories(src/tinyxml)
link_directories(${ABB_PATH}/lib)

rosbuild_add_executable(robot_node src/robot_node.cpp 
                        src/PracticalSocket/PracticalSocket.cpp
                        src/tinyxml/tinystr.cpp
                        src/tinyxml/tinyxml.cpp
                        src/tinyxml/tinyxmlerror.cpp
                        src/tinyxml/tinyxmlparser.cpp)
target_link_libraries(robot_node -lABBInterpreter)

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

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


#rosbuild_add_executable(regrasp5 src/regrasp5.cpp)
#rosbuild_add_executable(E2F src/E2F.cpp)
#rosbuild_add_executable(F2E2F src/F2E2F.cpp)
rosbuild_add_executable(F2E2FReduced src/F2E2FReduced.cpp)
#rosbuild_add_executable(BouncelessF2E src/BouncelessF2E.cpp)
rosbuild_add_executable(FlipInHand src/FlipInHand.cpp)
##rosbuild_add_executable(FlipInHandLoop src/FlipInHandLoop.cpp)
#rosbuild_add_executable(FlipInHandOpp src/FlipInHandOpp.cpp)
rosbuild_add_executable(FlipInAir src/FlipInAir.cpp)
#rosbuild_add_executable(FlipInAirLoop src/FlipInAirLoop.cpp)
rosbuild_add_executable(topple src/topple.cpp)
#rosbuild_add_executable(toppleLoop src/toppleLoop.cpp)
#rosbuild_add_executable(NewCylRot src/NewCylRot.cpp)
#rosbuild_add_executable(E2F_Opt src/E2F_Opt.cpp)
rosbuild_add_executable(PushTranslate src/PushTranslate.cpp)
#rosbuild_add_executable(hand_smoothening src/hand_smoothening.cpp)
rosbuild_add_executable(GroundRoll src/GroundRoll.cpp)
#rosbuild_add_executable(GroundRollLoop src/GroundRollLoop.cpp)
rosbuild_add_executable(RollInHand src/RollInHand.cpp)
#rosbuild_add_executable(RollInHandLoop src/RollInHandLoop.cpp)
rosbuild_add_executable(VertCyl src/VertCyl.cpp)
#rosbuild_add_executable(VertCylLoop src/VertCylLoop.cpp)
rosbuild_add_executable(RotateInFingers src/RotateInFingers.cpp)
rosbuild_add_executable(RotateInFingersLoop src/RotateInFingersLoop.cpp)
##rosbuild_add_executable(Sequence src/Sequence.cpp)
##rosbuild_add_executable(HandVideo src/HandVideo.cpp)
##rosbuild_add_executable(F2EHD src/F2EHD.cpp)
##rosbuild_add_executable(hand_OCTest src/hand_OCTest.cpp)
#rosbuild_add_executable(NewTopple src/NewTopple.cpp)
rosbuild_add_executable(PushInEnvelopingFixture src/PushInEnvelopingFixture.cpp)
#
rosbuild_add_executable(ABBCylDroop src/ABBCylDroop.cpp)
#rosbuild_add_executable(ABBBlackDroop src/ABBBlackDroop.cpp)
rosbuild_add_executable(ABBBlackDroopNew src/ABBBlackDroopNew.cpp)
rosbuild_add_executable(ABBAssembly src/ABBAssembly.cpp)

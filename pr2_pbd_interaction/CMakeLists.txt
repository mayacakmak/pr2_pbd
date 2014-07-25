cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

# Init
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_roslaunch_check( launch/pbd_frontend.launch )
rosbuild_add_roslaunch_check( launch/pbd_backend.launch ROBOT=sim )
rosbuild_add_roslaunch_check( launch/pr2_interactive_manipulation_desktop.launch )
rosbuild_add_roslaunch_check( launch/simulated_robot.launch ROBOT=sim ROS_ENV_LOADER=/etc/ros/env.sh )

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

#rosbuild_add_executable(rwrc14_waypoint_manager src/rwrc14_waypoint_manager_ikuta.cpp)
#target_link_libraries(rwrc14_waypoint_manager visualize GPStoXY)
#rosbuild_add_executable(rwrc14_waypoint_manager src/rwrc14_waypoint_manager_tsukuba.cpp)
#target_link_libraries(rwrc14_waypoint_manager visualize GPStoXY)

rosbuild_add_library(GPStoXY src/GPStoXY/GPStoXY.cpp)
rosbuild_add_library(visualize src/Visualize_lib.cpp)

rosbuild_add_library(init_visu src/init_visu.cpp)
rosbuild_add_library(read_wp_file src/read_wp_file.cpp)
target_link_libraries(read_wp_file GPStoXY)

#rosbuild_add_executable(rwrc14_waypoint_manager src/rwrc14_waypoint_manager.cpp)
#target_link_libraries(rwrc14_waypoint_manager visualize init_visu read_wp_file)

rosbuild_add_executable(rwrc15_waypoint_manager src/rwrc15_waypoint_manager.cpp)
target_link_libraries(rwrc15_waypoint_manager visualize init_visu read_wp_file)
#rosbuild_add_executable(sgt15_waypoint_manager src/sgt15_waypoint_manager.cpp)
#target_link_libraries(sgt15_waypoint_manager visualize GPStoXY)
#rosbuild_add_executable(sgt14_waypoint_manager src/sgt14_waypoint_manager.cpp)
#target_link_libraries(sgt14_waypoint_manager visualize GPStoXY)

rosbuild_add_executable(path_smoother src/path_smoother.cpp)
target_link_libraries(path_smoother ${catkin_LIBRARIES})


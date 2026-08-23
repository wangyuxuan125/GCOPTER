cmake_minimum_required(VERSION 2.8.3)

project(gcopter)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

# Optional implementation of the Liu et al. (ICRA 2017) ellipsoid-based
# decomposition baseline. GCOPTER still builds with its original FIRI method
# when DecompUtil is not sourced.
find_package(decomp_util QUIET)
if(decomp_util_FOUND)
  add_definitions(-DGCOPTER_WITH_DECOMP_UTIL)
  message(STATUS "gcopter: Liu/DecompUtil corridor baseline enabled")
else()
  message(STATUS "gcopter: decomp_util not found; ellipsoid_decomp unavailable")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${DECOMP_UTIL_INCLUDE_DIRS}
    include
)

catkin_package()

add_executable(global_planning src/global_planning.cpp)

target_link_libraries(global_planning
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
)

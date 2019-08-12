cmake_minimum_required(VERSION 3.1)
project(transform)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
include_directories("/usr/include/eigen3")
add_compile_options(-std=c++11 -Wall)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(transform src/transform.cpp )
target_link_libraries(transform ${LIBS})

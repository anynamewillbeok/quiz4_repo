cmake_minimum_required (VERSION 3.0.0)

project (a2_example)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
set(BUILD_TESTS ON)

find_package(Threads)
find_package(Eigen3 REQUIRED NO_MODULE)

find_package(pfms2ros 3.1.1 QUIET)
if(NOT pfms2ros_FOUND)
    message(STATUS "pfms2ros 3.1.1 not found. Please run: sudo apt update")
endif()
find_package(pfms2ros 3.1.1 REQUIRED)

find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)

add_library(logger logger.cpp)

add_library(ranger SHARED ranger.cpp laser.cpp sonar.cpp)
target_link_libraries(ranger pfmsconnector)

add_library(controller SHARED controller.cpp ackerman.cpp skidsteer.cpp)
target_link_libraries(controller pfmsconnector audi Eigen3::Eigen)

add_library(mission SHARED mission.cpp)
target_link_libraries(mission controller ranger pfmsconnector audi)

add_executable(a2 main.cpp)
target_link_libraries(a2 logger ranger controller mission pfmsconnector)

if(BUILD_TESTS)
   add_subdirectory(test)
endif()

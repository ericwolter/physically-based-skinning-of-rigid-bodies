cmake_minimum_required(VERSION 2.8)

add_definitions(-DUSE_DOUBLES)

include_directories(${CMAKE_CURRENT_LIST_DIR})

add_subdirectory(Simulation)

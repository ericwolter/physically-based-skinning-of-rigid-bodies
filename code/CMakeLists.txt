cmake_minimum_required(VERSION 2.8)

add_definitions(-DUSE_DOUBLES)

include_directories(${CMAKE_CURRENT_LIST_DIR})

# external deps
add_subdirectory(deps/AntTweakBar)

# my own code
add_subdirectory(Simulation)

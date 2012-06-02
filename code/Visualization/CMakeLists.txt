project(Visualization)

include_directories(include)
include_directories(${CMAKE_SOURCE_DIR}/Math2/include)
include_directories(${CMAKE_SOURCE_DIR}/deps/AntTweakBar/include)
include_directories(${CMAKE_SOURCE_DIR}/deps/glfw/include)

file(GLOB_RECURSE Visualization_SOURCES src/*.cpp)

add_library(Visualization STATIC ${Visualization_SOURCES})

find_library(GLFW_LIBRARY libglfw.a PATHS ${CMAKE_SOURCE_DIR}/deps/glfw/lib/*)

target_link_libraries(Visualization AntTweakBar Math2 ${GLFW_LIBRARY})

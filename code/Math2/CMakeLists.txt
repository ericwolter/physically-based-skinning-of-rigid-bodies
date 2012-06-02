project(Math2)

include_directories(include)

file(GLOB_RECURSE Math2_SOURCES src/*.cpp)

add_library(Math2 STATIC ${Math2_SOURCES})
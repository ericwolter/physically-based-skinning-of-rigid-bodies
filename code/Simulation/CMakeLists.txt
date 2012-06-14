project(Simulation)

include_directories(${CMAKE_SOURCE_DIR}/deps/eigen)
include_directories(${CMAKE_SOURCE_DIR}/deps/glfw/include)

if(APPLE)
	include_directories(/System/Library/Frameworks)
	find_library(COCOA_LIBRARY Cocoa)
	find_library(OpenGL_LIBRARY OpenGL)
	find_library(GLFW_LIBRARY libglfw.a PATHS ${CMAKE_SOURCE_DIR}/deps/glfw/lib/cocoa/)
	mark_as_advanced(COCOA_LIBRARY GLUT_LIBRARY OpenGL_LIBRARY)
	set(EXTRA_LIBS ${COCOA_LIBRARY} ${OpenGL_LIBRARY} ${GLFW_LIBRARY})
endif()

file(COPY src/models DESTINATION ${CMAKE_BINARY_DIR}/Simulation/)
file(COPY src/shaders DESTINATION ${CMAKE_BINARY_DIR}/Simulation/)
file(GLOB_RECURSE Simulation_SOURCES src/*.cpp)

add_executable(Simulation ${Simulation_SOURCES})

target_link_libraries(Simulation ${EXTRA_LIBS})
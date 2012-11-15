cmake_minimum_required (VERSION 2.6)

project(Thesis)

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)

find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

find_package(Bullet REQUIRED)
include_directories(${Bullet_INCLUDE_DIRS})
link_directories(${Bullet_LIBRARY_DIRS})
add_definitions(${Bullet_DEFINITIONS})
if(NOT BULLET_FOUND)
    message(ERROR " BULLET_FOUND not found!")
endif(NOT BULLET_FOUND)

file(GLOB_RECURSE Thesis_SOURCES src/*.cpp)
file(GLOB_RECURSE Thesis_HEADERS src/*.h)
add_executable(Thesis ${Thesis_SOURCES} ${Thesis_HEADERS})

target_link_libraries(Thesis ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${BULLET_LIBRARIES})
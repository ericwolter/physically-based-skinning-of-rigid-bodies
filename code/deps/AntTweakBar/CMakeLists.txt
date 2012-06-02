project(AntTweakBar)

include_directories(include)

if(APPLE)
	add_definitions(-D_MACOSX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ObjC++")
	set(CMAKE_CXX_LINKER_FLAGS "${CMAKE_CXX_LINKER_FLAGS} -framework OpenGL -framework GLUT -framework AppKit")
	set(AntTweakBar_SOURCES src/TwColors.cpp src/TwFonts.cpp src/TwOpenGL.cpp src/TwBar.cpp src/TwMgr.cpp src/TwPrecomp.cpp src/LoadOGL.cpp src/TwEventGLFW.c src/TwEventGLUT.c src/TwEventSDL.c src/TwEventSDL12.c src/TwEventSDL13.c src/TwEventSFML.cpp)
endif()

add_library(AntTweakBar STATIC ${AntTweakBar_SOURCES})
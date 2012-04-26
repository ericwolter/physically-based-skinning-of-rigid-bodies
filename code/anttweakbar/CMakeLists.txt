include_directories(include)

add_library(anttweakbar SHARED
    src/LoadOGL.cpp
    src/LoadOGLCore.cpp
    src/TwBar.cpp
    src/TwColors.cpp
    src/TwEventGLFW.c
    src/TwEventGLUT.c
    src/TwEventSDL.c
    src/TwEventSDL12.c
    src/TwEventSDL13.c
    src/TwEventSFML.cpp
    src/TwEventWin.c
    src/TwFonts.cpp
    src/TwMgr.cpp
    src/TwOpenGL.cpp
    src/TwPrecomp.cpp   
	include/AntTweakBar.h
)

add_definitions(-DTW_EXPORTS)
add_definitions(-D_USRDLL)  
add_definitions(-D_CRT_SECURE_NO_WARNINGS) 

set_target_properties(anttweakbar PROPERTIES FOLDER "External libraries")
 
add_custom_command(
   TARGET anttweakbar
   POST_BUILD
   COMMAND xcopy /r /y \"$\(TargetPath\)" \"$\(TargetDir\)..\\..\\bin\\$\(ConfigurationName\)\\"
)                        


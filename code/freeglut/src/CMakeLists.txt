
ADD_LIBRARY(freeglut SHARED
      freeglut_callbacks.c
      freeglut_cursor.c
      freeglut_display.c
      freeglut_ext.c
      freeglut_font.c
      freeglut_font_data.c
      freeglut_gamemode.c
      freeglut_geometry.c
      freeglut_glutfont_definitions.c
      freeglut_init.c
      freeglut_internal.h
      freeglut_joystick.c
      freeglut_main.c
      freeglut_menu.c
      freeglut_misc.c
      freeglut_overlay.c
      freeglut_state.c
      freeglut_stroke_mono_roman.c
      freeglut_stroke_roman.c
      freeglut_structure.c
      freeglut_teapot.c
      freeglut_teapot_data.h
      freeglut_videoresize.c
      freeglut_window.c
	  freeglut_input_devices.c
	  freeglut_spaceball.c
      ../include/GL/freeglut.h
      ../include/GL/freeglut_ext.h
      ../include/GL/freeglut_std.h
      ../include/GL/glut.h
	  
	  CMakeLists.txt
)

ADD_DEFINITIONS(-DFREEGLUT_EXPORTS)         
ADD_DEFINITIONS(-D_USRDLL)  
ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS) 

IF (WIN32)
	ADD_CUSTOM_COMMAND(
	   TARGET freeglut
	   POST_BUILD
	   COMMAND xcopy /r /y \"$\(TargetPath\)" \"$\(TargetDir\)..\\..\\bin\\$\(ConfigurationName\)\\"
	)                        
ENDIF (WIN32)

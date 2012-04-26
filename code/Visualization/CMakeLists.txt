INCLUDE_DIRECTORIES(${PROJECT_PATH}/freeglut/include)

SET(VIS_HEADER_FILES
	${PROJECT_PATH}/Visualization/MiniGL.h
	) 

	
SET(VIS_SOURCE_FILES
	${PROJECT_PATH}/Visualization/MiniGL.cpp                       
	) 
	

SET(VIS_FILES
	${VIS_HEADER_FILES}
	${VIS_SOURCE_FILES}
	) 
	
# A macro for the source groups
MACRO(VIS_SOURCE_GROUPS)
	SOURCE_GROUP("Header Files\\Visualization" FILES ${VIS_HEADER_FILES})
	SOURCE_GROUP("Source Files\\Visualization" FILES ${VIS_SOURCE_FILES})
ENDMACRO(VIS_SOURCE_GROUPS)




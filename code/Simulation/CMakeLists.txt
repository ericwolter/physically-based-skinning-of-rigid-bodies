SET(SIMULATION_LINK_LIBRARIES Math2 freeglut opengl32.lib glu32.lib anttweakbar)
SET (SIMULATION_DEPENDENCIES Math2 freeglut anttweakbar)

# PROJECT(ClothModel)
ADD_EXECUTABLE(ClothModel
	  ClothModel.cpp
	  ParticleSystem.cpp
	  ParticleSystem.h  
	  SimulationStep.cpp
	  SimulationStep.h
	  TimeManager.cpp
	  TimeManager.h
	  ${VIS_FILES}                    
	  ${PROJECT_PATH}/Common/Config.h			  
	  ${PROJECT_PATH}/Common/IDFactory.h			  
	  ${PROJECT_PATH}/Common/IDFactory.cpp
	  ${PROJECT_PATH}/Common/StringTools.h			  
	  ${PROJECT_PATH}/Common/StringTools.cpp
	  ${PROJECT_PATH}/Common/timing.h			  	  
	  ${PROJECT_PATH}/Common/timing.cpp
	  CMakeLists.txt
)
ADD_DEPENDENCIES(ClothModel ${SIMULATION_DEPENDENCIES})
TARGET_LINK_LIBRARIES(ClothModel ${SIMULATION_LINK_LIBRARIES})
VIS_SOURCE_GROUPS()

/*
 * IBDS - Impulse-Based Dynamic Simulation Library
 * Copyright (c) 2003-2008 Jan Bender http://www.impulse-based.de
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * Jan Bender - Jan.Bender@impulse-based.de
 */


#include "Common/Config.h"
#include "Visualization/MiniGL.h"
#include "GL/glut.h"
#include "TimeManager.h"
#include "Math/SimMath.h"
#include "Common/StringTools.h"
#include "Common/timing.h"

// Enable memory leak detection
#ifdef _DEBUG
	#define new DEBUG_NEW 
#endif

using namespace IBDS;
using namespace std;

void timeStep ();
void buildModel ();
void render ();
void cleanup();


// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS
	USE_TIMESTEP_TIMING(Timing::m_dontPrintTimes = true;);

	// OpenGL
	MiniGL::init (argc, argv, 800, 600, 0, 0, "MiniGL");
	MiniGL::initLights ();
	MiniGL::setClientIdleFunc (50, timeStep);				

	buildModel ();

	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0f, 1.0f, 100.0f, Vector3D (3.0, 1.0, 10.0), Vector3D (3.0, -0.3, 0.0));

	glutMainLoop ();	

	cleanup ();

	USE_TIMESTEP_TIMING(printAverageTimes());
	
	return 0;
}

void cleanup()
{
}

void timeStep ()
{
	START_TIMING("timeStep");

	TimeManager *tm = TimeManager::getCurrent ();
	const Real h = tm->getTimeStepSize();

	// Simulation code

	tm->setTime(tm->getTime() + h);

	STOP_TIMING_AVG;
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (0.01);

	// Create simulation model
}


void render ()
{
	MiniGL::coordinateSystem();
	
	// Draw simulation model

	MiniGL::drawTime( TimeManager::getCurrent ()->getTime ());
}


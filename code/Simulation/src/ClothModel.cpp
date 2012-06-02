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
/*
#define _USE_MATH_DEFINES 
#include <sstream>
#include <string>
#include <cmath>
#include <ctime>
#include <Common/timing.h>
#include <Visualization/MiniGL.h>
#include "ParticleSystem.h"
#include "SimulationStep.h"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include <GL/glut.h>
#endif

// Enable memory leak detection
#ifdef _DEBUG
#define new DEBUG_NEW 
#endif

using namespace IBDS;
using namespace std;

void timeStep ();
void buildModel ();
void render ();
void exit ();
void performStep();


struct Configuration 
{
	int ode;
	Real h;
	Real structSpringConstant;
	Real structFriction;
	Real shearSpringConstant;
	Real shearFriction;
	Real bendSpringConstant;
	Real bendFriction;
	bool drawLines;
};

#define ROWS 10
#define COLS 10

Configuration euler = { 0, 0.002, 500.0, 15.0, 50.0, 10.0, 20.0, 10.0, false };
Configuration rk4 = { 1, 0.01, 800.0, 20.0, 200.0, 10.0, 50.0, 10.0, true };
Configuration *activeConfig = &rk4;
bool doStep = false;

ParticleSystem ps;
SimulationStep step(&ps, activeConfig->h, activeConfig->ode);

// main 
int main( int argc, char **argv )
{
	srand( (unsigned) time (NULL));

	// OpenGL
	MiniGL::init (argc, argv, 800, 600, 0, 0, "MiniGL");

	return 0;
	// MiniGL::initLights ();
	// MiniGL::setClientIdleFunc (50, timeStep);	
	// MiniGL::setKeyFunc(0, ' ', performStep);	

	// buildModel ();

	// MiniGL::setClientSceneFunc(render);			
	// MiniGL::setViewport (40.0f, 1.0f, 1000.0f, Vector3D (3.0, 3.0, 10.0), Vector3D (3.0, -0.3, 0.0));

	// glutMainLoop();

	//USE_TIMESTEP_TIMING(printAverageTimes());

	// REPORT_MEMORY_LEAKS

	// return 0;
}


void timeStep ()
{
	if (doStep)
	{
		//for (int i=0; i < 4; i++)
		step.timeStep();
	}
}

void performStep()
{
	doStep = !doStep;
}

void buildModel ()
{
	// Particles
	ps.setNumberOfParticles(ROWS*COLS);
	Real *m = ps.getMasses();
	for (int i=0; i < ROWS; i++)
	{
		for (int j=0; j < COLS; j++)
		{
			Real *x = ps.getPos(i*COLS+j);
			x[0] = j;
			x[1] = 0.0;
			x[2] = -i;
			
			m[i] = 1.0;
		}
	}
	//for (int j=0; j < COLS; j++)
	//	m[j] = 0.0;
	m[0] = 0.0;
	m[COLS-1] = 0.0;

	//Real *v = ps.getVelocities();
	//v[350] = 50;
	//v[200] = 50;
	//v[227] = 50;
	//v[228] = 25;
	//v[229] = -20;

	// Structural springs
	// Horizontal
	for (int i=0; i < ROWS; i++)
	{
		for (int j=0; j < COLS-1; j++)
		{
			ps.addSpring(i*COLS+j, i*COLS+j+1, activeConfig->structSpringConstant, activeConfig->structFriction);
		}
	}

	// Vertical
	for (int i=0; i < ROWS-1; i++)
	{
		for (int j=0; j < COLS; j++)
		{
			ps.addSpring(i*COLS+j, (i+1)*COLS+j, activeConfig->structSpringConstant, activeConfig->structFriction);
		}
	}

	// Shearing springs
	for (int i=0; i < ROWS-1; i++)
	{
		for (int j=0; j < COLS-1; j++)
		{
			ps.addSpring(i*COLS+j, (i+1)*COLS+j+1, activeConfig->shearSpringConstant, activeConfig->shearFriction);
			ps.addSpring((i+1)*COLS+j, i*COLS+j+1, activeConfig->shearSpringConstant, activeConfig->shearFriction);
		}
	}

	// Bending springs
	for (int i=0; i < ROWS; i++)
	{
		for (int j=0; j < COLS-2; j++)
		{
			ps.addSpring(i*COLS+j, i*COLS+j+2, activeConfig->bendSpringConstant, activeConfig->bendFriction);
		}
	}
	for (int i=0; i < ROWS-2; i++)
	{
		for (int j=0; j < COLS; j++)
		{
			ps.addSpring(i*COLS+j, (i+2)*COLS+j, activeConfig->bendSpringConstant, activeConfig->bendFriction);
		}
	}
}



void render ()
{
	MiniGL::coordinateSystem();
	
	float col [4] = {0,0,1,1};
	for (int i=0; i < ps.getNumberOfParticles(); i++)
	{
		Real *pos = ps.getPos(i);
		Vector3D t(pos[0], pos[1], pos[2]);
		MiniGL::drawSphere(&t, 0.1f, col);
	}

	if (activeConfig->drawLines)
	{
		float col2 [4] = {1,0,0,1};
		for (int i=0; i < (ROWS*(COLS-1)) + (COLS*(ROWS-1)); i++)
		{
			ParticleSystem::Spring *spring = ps.getSpring(i);
			Real *p1 = ps.getPos(spring->index1);
			Real *p2 = ps.getPos(spring->index2);
			MiniGL::drawVector(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2], 1.0f, col2);
		}
	}

	float triCol [4] = {0.4f,0.6f,0.8f,1.0f};
	for (int i=0; i < ROWS-1; i++)
	{
		for (int j=0; j < COLS-1; j++)
		{
			Real *a = ps.getPos(i*COLS+j);
			Real *b = ps.getPos(i*COLS+j+1);
			Real *c = ps.getPos((i+1)*COLS+j);
			Real *d = ps.getPos((i+1)*COLS+j+1);
			const Vector3D vecA(a[0], a[1], a[2]);
			const Vector3D vecB(b[0], b[1], b[2]);
			const Vector3D vecC(c[0], c[1], c[2]);
			const Vector3D vecD(d[0], d[1], d[2]);
			const Vector3D norm1 = (vecB-vecA) ^ (vecC - vecA);
			const Vector3D norm2 = (vecD-vecB) ^ (vecC - vecB);
			MiniGL::drawTriangle(vecA, vecB, vecC, norm1, triCol);
			MiniGL::drawTriangle(vecB, vecD, vecC, norm2, triCol);
		}
	}

	MiniGL::drawTime(step.getTime());
}
*/



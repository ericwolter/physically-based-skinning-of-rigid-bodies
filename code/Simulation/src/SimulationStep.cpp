#include <Common/timing.h>
#include <Math2/Vec.h>
#include "SimulationStep.h"

using namespace IBDS;

#define GRAVITY -9.81


SimulationStep::SimulationStep(ParticleSystem *p, const Real timeStepSize, const int ode)
{	
	time = 0.0;
	h = timeStepSize;
	ps = p;
	solver = ode;
}

SimulationStep::~SimulationStep(void)
{
}

void SimulationStep::timeStep(void)
{
	//START_TIMING("Time Step");
	switch (solver)
	{
		case 0: 
			eulerStep();
			break;
		case 1: 
			rk4Step();
			break;
	}
	//STOP_TIMING_AVG;
	time += h;
}


void SimulationStep::setTimeStepSize(Real stepSize)
{
	h = stepSize;
}

Real SimulationStep::getTime() const
{
	return time;
}

void SimulationStep::clearForces()
{
	Real *f = ps->getForces();
	Real *m = ps->getMasses();
	for (int i=0; i < ps->getNumberOfParticles(); i++)
	{
		if (m[i] != 0.0)
		{
			f[3*i] = 0.0;
			f[3*i+1] = GRAVITY / m[i];
			f[3*i+2] = 0.0;
		}
	}
}

void SimulationStep::computeForces(Real *xv)
{
	for (int i=0; i < ps->getNumberOfSprings(); i++)
	{	
		ParticleSystem::Spring *spring = ps->getSpring(i);
		Real *pos1 = &xv[6*spring->index1];
		Vector3D p1(pos1[0], pos1[1], pos1[2]);
		Real *pos2 = &xv[6*spring->index2];
		Vector3D p2(pos2[0], pos2[1], pos2[2]);
		Vector3D deltaX = p1-p2;
		Real dl = deltaX.length();
		if (dl < 1.0e-4)
		{
			dl = 1.0e-4;
		}
		const Real springForce = spring->springConstant * (dl - spring->length);

		Real *vel1 = &xv[6*spring->index1+3];
		Vector3D v1(vel1[0], vel1[1], vel1[2]);
		Real *vel2 = &xv[6*spring->index2+3];
		Vector3D v2(vel2[0], vel2[1], vel2[2]);
		Vector3D deltaV = v1-v2;
		const Real frictionForce = spring->friction * ((deltaV*deltaX)/dl);

		Vector3D force = -(springForce + frictionForce)*(deltaX * (1.0 / dl));
		Real *f1 = ps->getForce(spring->index1);
		f1[0] += force[0];
		f1[1] += force[1];
		f1[2] += force[2];
		Real *f2 = ps->getForce(spring->index2);
		f2[0] -= force[0];
		f2[1] -= force[1];
		f2[2] -= force[2];
	}
}

void SimulationStep::eulerStep()
{
	clearForces();
	computeForces(ps->getState());

	// Compute derivative
	Real deriv[ParticleSystem::MAX_PARTICLES*6];
	derive(ps->getState(), &deriv[0]);
	// h * x'
	for (int i=0; i < 6*ps->getNumberOfParticles(); i++)
	{
		deriv[i] *= h;
	}

	for (int i=0; i < ps->getNumberOfParticles(); i++)
	{
		Real *pos = ps->getPos(i);
		pos[0] += deriv[6*i];
		pos[1] += deriv[6*i+1];
		pos[2] += deriv[6*i+2];
		Real *vel = ps->getVel(i);
		vel[0] += deriv[6*i+3];
		vel[1] += deriv[6*i+4];
		vel[2] += deriv[6*i+5];
	}
}

void SimulationStep::rk4Step()
{
	const int n = 6*ps->getNumberOfParticles();
	Real *currentState = ps->getState();
	Real k1[ParticleSystem::MAX_PARTICLES*6];
	Real k2[ParticleSystem::MAX_PARTICLES*6];
	Real k3[ParticleSystem::MAX_PARTICLES*6];
	Real k4[ParticleSystem::MAX_PARTICLES*6];
	Real deriv[ParticleSystem::MAX_PARTICLES*6];

	// Compute k1
	clearForces();
	computeForces(&currentState[0]);
	derive(&currentState[0], &deriv[0]);
	scale(n, &deriv[0], h, &k1[0]);

	// Compute k2
	clearForces();
	addScale(n, &k1[0], 0.5, &currentState[0], &k2[0]);
	computeForces(&k2[0]);
	derive(&k2[0], &deriv[0]);
	scale(n, &deriv[0], h, &k2[0]);

	// Compute k3
	clearForces();
	addScale(n, &k2[0], 0.5, &currentState[0], &k3[0]);
	computeForces(&k3[0]);
	derive(&k3[0], &deriv[0]);
	scale(n, &deriv[0], h, &k3[0]);

	// Compute k4
	clearForces();
	addScale(n, &k3[0], 1.0, &currentState[0], &k4[0]);
	computeForces(&k4[0]);
	derive(&k4[0], &deriv[0]);
	scale(n, &deriv[0], h, &k4[0]);

	for (int i=0; i < n; i++)
	{
		currentState[i] += 1.0/6.0*k1[i] + 1.0/3.0*k2[i] + 1.0/3.0*k3[i] + 1.0/6.0*k4[i];
	}
}


/** State is a vector of positions and velocities: (x1, y1, z1, vx1, vy1, vz1, ...)
 */
void SimulationStep::derive(Real *state, Real *dst)
{
	Real *f = ps->getForces();
	Real *m = ps->getMasses();
	for(int i=0; i < ps->getNumberOfParticles(); i++)
	{	
		if (m[i] != 0.0)
		{
			*(dst++) = state[6*i+3]; /* xdot = v */
			*(dst++) = state[6*i+4];
			*(dst++) = state[6*i+5];
			*(dst++) = f[3*i]/m[i]; /* vdot = f/m */
			*(dst++) = f[3*i+1]/m[i];
			*(dst++) = f[3*i+2]/m[i];
		}
		else
		{
			for (int j=0; j < 6; j++)
				*(dst++) = 0.0;
		}
	}
}

void SimulationStep::scale(int n, Real *src, Real factor, Real *dst)
{
	for(int i=0; i < n; i++)
	{
		*(dst++) = *(src++) * factor;
	}
}

void SimulationStep::addScale(int n, Real *src1, Real factor1, Real *src2, Real *dst)
{
	for(int i=0; i < n; i++)
	{
		*(dst++) = *(src1++) * factor1 + *(src2++);
	}
}

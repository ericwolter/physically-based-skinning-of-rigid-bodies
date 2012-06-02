#include "ParticleSystem.h"

using namespace IBDS;


ParticleSystem::ParticleSystem()
{	
	numberOfParticles = 0;
	numberOfSprings = 0;

	// Init vectors
	for (int i=0; i < MAX_PARTICLES; i++)
	{
		m[i] = 1.0;

		for (int j=0; j < 3; j++)
		{
			xv[6*i+j] = 0.0;
			xv[6*i+j+3] = 0.0;
			f[3*i+j] = 0.0;
		}
	}
}

ParticleSystem::~ParticleSystem(void)
{
	numberOfParticles = 0;
}

Real *ParticleSystem::getPos(int i)
{
	return &xv[6*i];
}

Real *ParticleSystem::getVel(int i)
{
	return &xv[6*i+3];
}

Real *ParticleSystem::getState()
{
	return &xv[0];
}

Real *ParticleSystem::getForces()
{
	return &f[0];
}

Real *ParticleSystem::getForce(int i)
{
	return &f[3*i];
}

Real *ParticleSystem::getMasses()
{
	return &m[0];
}

Real *ParticleSystem::getMass(int i)
{
	return &m[i];
}

int ParticleSystem::getNumberOfParticles() const
{
	return numberOfParticles;
}

int ParticleSystem::getNumberOfSprings() const
{
	return numberOfSprings;
}

void ParticleSystem::setNumberOfParticles(int num)
{
	numberOfParticles = num;
}

void ParticleSystem::addSpring(const int i, const int j, const Real springConstant, const Real friction)
{
	springs[numberOfSprings].springConstant = springConstant;
	springs[numberOfSprings].friction = friction;
	springs[numberOfSprings].index1 = i;
	springs[numberOfSprings].index2 = j;
	Real *pos = getPos(i);
	Vector3D p1(pos[0], pos[1], pos[2]);
	pos = getPos(j);
	Vector3D p2(pos[0], pos[1], pos[2]);
	springs[numberOfSprings].length = (p1-p2).length();
	numberOfSprings++;
}

ParticleSystem::Spring *ParticleSystem::getSpring(int i)
{
	return &springs[i];
}
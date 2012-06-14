#ifndef __SIMULATION_H__
#define __SIMULATION_H__

#include "RigidBody.h"

class Simulation
{
public:
	Simulation();
	~Simulation();

	std::vector<RigidBody> bodies;
};

#endif
#ifndef __SIMULATION_H__
#define __SIMULATION_H__

#include <vector>
#include "RigidBody.h"

class Simulation
{
public:
	Simulation();
	~Simulation();

	std::vector<RigidBody> bodies;

	void step(float dt);
	void computeForceAndTorque(float dt, RigidBody &body);

	void render(Affine3f cameraTransform, GLuint modelToCameraMatrixUnif);
};

#endif
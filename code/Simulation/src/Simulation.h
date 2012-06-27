#ifndef __SIMULATION_H__
#define __SIMULATION_H__

#include <vector>

#include "RigidBody.h"
#include "Contact.h"

class Simulation
{
public:
	Simulation();
	~Simulation();

	void step(float dt);
	bool findSeperatingPlane(RigidBody &a, RigidBody &b);
	bool testSeperatingPlane(MatrixXf &plane, RigidBody &body);

	void render(Affine3f cameraTransform, GLuint modelToCameraMatrixUnif);

	std::vector<RigidBody> bodies;

private:
	std::vector<Contact> contacts;
	void restore();
};

#endif
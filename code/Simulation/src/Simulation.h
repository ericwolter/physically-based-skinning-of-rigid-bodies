#ifndef __SIMULATION_H__
#define __SIMULATION_H__

#include <vector>

#include "RigidBody.h"
#include "Contact.h"

enum CollisionType {
	CLEAR,
	PENETRATING,
	COLLIDING
};

struct CollisionState {
	CollisionType type;
	std::vector<Contact> contacts;
};

class Simulation
{
public:
	Simulation();
	~Simulation();

	void step(float dt);
	CollisionState findSeperatingPlane(RigidBody &a, RigidBody &b);
	CollisionState areColliding(RigidBody &a, RigidBody &b);

	void render(Affine3f cameraTransform, GLuint modelToCameraMatrixUnif);

	std::vector<RigidBody> bodies;

private:
	std::vector<Contact> contacts;
	void restore();
};

#endif
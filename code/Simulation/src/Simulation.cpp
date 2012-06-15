#include "Simulation.h"

Simulation::Simulation() {}
Simulation::~Simulation() {}

void Simulation::step(float dt)
{
	std::vector<RigidBody>::iterator body;
    for (body = this->bodies.begin(); body != this->bodies.end(); body++)
    {
    	computeForceAndTorque(dt, (*body));

    	body->integrate(dt);
    }
}

void Simulation::computeForceAndTorque(float dt, RigidBody &body)
{

}

void Simulation::render(Affine3f cameraTransform, GLuint modelToCameraMatrixUnif)
{
	std::vector<RigidBody>::iterator body;
    for (body = this->bodies.begin(); body != this->bodies.end(); body++)
    {
    	body->render(cameraTransform, modelToCameraMatrixUnif);
    }
}

#include <iostream>
#include <assert.h>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

#include "../src/RigidBody.h"
#include "../src/ObjLoader.h"
#include "../src/Simulation.h"

ObjLoader objLoader;
string base_path("");
Vector4f bodyColor;
RigidBody cube;
RigidBody plane;
Simulation sim;

string ExtractDirectory(const string &path)
{
    return path.substr(0, path.find_last_of( '/' ) + 1);
}

void findSeperatingPlane_CubeIsFarAwayFromPlane_CollisionShouldBeClear() {
	cube = RigidBody(objLoader.parseObj(base_path + "models/cube.obj"), (MatrixXf(3, 1) << 0.0f, 2.0f, 0.0f).finished(), bodyColor);
	plane = RigidBody(objLoader.parseObj(base_path + "models/plane.obj"), Vector3f::Zero(), bodyColor, true);

	struct CollisionState collision = sim.areColliding(cube, plane);
	assert(collision.type == CLEAR);
	assert(collision.contacts.size() == 0);
}

void findSeperatingPlane_CubeIsInsidePlane_CollisionPenetrate() {
	cube = RigidBody(objLoader.parseObj(base_path + "models/cube.obj"), (MatrixXf(3, 1) << 0.0f, -1.5f, 0.0f).finished(), bodyColor);
	plane = RigidBody(objLoader.parseObj(base_path + "models/plane.obj"), Vector3f::Zero(), bodyColor, true);

	struct CollisionState collision = sim.areColliding(cube, plane);
	assert(collision.type == PENETRATING);
}

void findSeperatingPlane_CubeIsExactlyTouchingThePlane_CollisionShouldProvide4ContactPoints() {
	cube = RigidBody(objLoader.parseObj(base_path + "models/cube.obj"), (MatrixXf(3, 1) << 10.0f, 1.0f, 10.0f).finished(), bodyColor);
	plane = RigidBody(objLoader.parseObj(base_path + "models/plane.obj"), Vector3f::Zero(), bodyColor, true);

	struct CollisionState collision = sim.areColliding(cube, plane);
	assert(collision.type == COLLIDING);
	assert(collision.contacts.size() == 4);
}

void findSeperatingPlane_CubeIsAlmostTouchingThePlane_CollisionShouldProvide4ContactPoints() {
	cube = RigidBody(objLoader.parseObj(base_path + "models/cube.obj"), (MatrixXf(3, 1) << 1.0f, 0.0175773f, 0.0f).finished(), bodyColor);
	plane = RigidBody(objLoader.parseObj(base_path + "models/plane.obj"), Vector3f::Zero(), bodyColor, true);

	struct CollisionState collision = sim.areColliding(cube, plane);
	assert(collision.type == COLLIDING);
	assert(collision.contacts.size() == 4);
}

void findSeperatingPlane_CubeIsExactlyTouchingThePlaneComparingPlaneAgainstCube_CollisionShouldProvide4ContactPoints() 
{
	cube = RigidBody(objLoader.parseObj(base_path + "models/cube.obj"), (MatrixXf(3, 1) << 1.0f, 1.0f, 1.0f).finished(), bodyColor);
	plane = RigidBody(objLoader.parseObj(base_path + "models/plane.obj"), Vector3f::Zero(), bodyColor, true);

	struct CollisionState collision = sim.areColliding(plane, cube);
	assert(collision.type == COLLIDING);
	assert(collision.contacts.size() == 4);
}

int main(int argc, const char *argv[])
{
	base_path = ExtractDirectory(argv[0]);

    bodyColor[0] = 220.0f / 255.0f;
    bodyColor[1] = 50.0f / 255.0f;
    bodyColor[2] = 40.0f / 255.0f;
    bodyColor[3] = 1.0f;

    findSeperatingPlane_CubeIsFarAwayFromPlane_CollisionShouldBeClear();
    findSeperatingPlane_CubeIsInsidePlane_CollisionPenetrate();
    findSeperatingPlane_CubeIsExactlyTouchingThePlane_CollisionShouldProvide4ContactPoints();
    findSeperatingPlane_CubeIsAlmostTouchingThePlane_CollisionShouldProvide4ContactPoints();
    findSeperatingPlane_CubeIsExactlyTouchingThePlaneComparingPlaneAgainstCube_CollisionShouldProvide4ContactPoints();
}
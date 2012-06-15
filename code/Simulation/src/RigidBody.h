#ifndef __RIGIDBODY_H__
#define __RIGIDBODY_H__

#include "Polyhedron.h"
#include <Eigen/Dense>
using namespace Eigen;

#define GLFW_INCLUDE_GL3
#define GLFW_NO_GLU
#include <GL/glfw.h>

class RigidBody
{
public:
	RigidBody();
	RigidBody(Polyhedron polyhedron, Vector3f worldPosition, Vector4f color, bool fixed=false);
	~RigidBody();

	Polyhedron mesh;

	void render(Affine3f cameraTransform, GLuint modelToCameraMatrixUnif);
	void integrate(float dt);
private:
	// rendering variables
	GLuint vertexArrayObject;
	GLuint vertexBufferObject;
	GLuint colorBufferObject;
	GLuint indexBufferObject;
	GLuint normalBufferObject;

	// constant quantities
	float massInverse;
	Matrix3f inertiaTensorInverse;

	// state quantities
	Vector3f position;
	Quaternionf orientation;
	Vector3f linearMomentum;
	Vector3f angularMomentum;

	// derived quantities (auxiliary variables)
	Vector3f linearVelocity;
	Vector3f angularVelocity;
	Matrix3f inertia;
	Matrix3f inertiaInverse;

	// computed quantities
	Vector3f force;
	Vector3f torque;
};

#endif
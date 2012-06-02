#ifndef __POLYHEDRON_H__
#define __POLYHEDRON_H__

#include <vector>
#include <Eigen/Dense>
using namespace Eigen;

#include "Face.h"

class Polyhedron
{
public:
	Polyhedron();
	~Polyhedron();

	std::vector<Face> faces;
	double mass;
	Vector3d centerOfMass;
	Matrix3d inertiaTensor;

	void computeProperties();
};

#endif
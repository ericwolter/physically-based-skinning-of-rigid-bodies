#include "Polyhedron.h"
#include <iostream>

using namespace std;

Polyhedron::Polyhedron() {}
Polyhedron::~Polyhedron() {}

void Polyhedron::computeProperties() {
	this->inertiaTensor = Matrix3d::Zero();
	this->mass = 0.0;
	this->centerOfMass = Vector3d::Zero();

	Matrix3d covariance = Matrix3d::Zero();

	printf("Computing properties of polyhedron\n");
	vector<Face>::iterator face;
	for(face = this->faces.begin(); face != this->faces.end(); face++) {
		Matrix3d single_covariance;
		double single_mass;
		Vector3d single_centerOfMass;
		(*face).computeProperties(single_covariance, single_mass, single_centerOfMass);

		covariance += single_covariance;
		this->centerOfMass = (this->centerOfMass*this->mass + single_centerOfMass * single_mass) / (this->mass + single_mass);
		this->mass += single_mass;
	}

	// translate to center of mass
	Vector3d offset = (-1) * this->centerOfMass;
	Matrix3d o1 = offset * this->centerOfMass.transpose();
	Matrix3d o2 = this->centerOfMass * offset.transpose();
	Matrix3d o3 = offset * offset.transpose();
	covariance = covariance + this->mass * (o1 + o2 + o3);

	this->inertiaTensor = Matrix3d::Identity() * covariance.trace() - covariance;
	std::cout << "inertiaTensor: " << std::endl;
	std::cout << this->inertiaTensor << std::endl;
};


#ifndef __FACE_H__
#define __FACE_H__

#include <vector>
#include <Eigen/Dense>

using namespace Eigen;

class Face
{
public:
	Face();
	~Face();

	std::vector<Vector3d> vertices;
	Vector3d normal;

	void computeProperties(Matrix3d& covariance, double& mass, Vector3d& centerOfMass);
};

#endif
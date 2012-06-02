#ifndef __OBJLOADER_H__
#define __OBJLOADER_H__

#include <string>
#include <vector>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

class Polyhedron;
class Face;

class ObjLoader
{
public:
	ObjLoader();
	~ObjLoader();

	void parseObj(string filename);
private:
	vector<Vector3d> vertices;
	vector<Vector3d> normals;
};

#endif
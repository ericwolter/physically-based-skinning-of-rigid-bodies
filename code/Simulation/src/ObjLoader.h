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

    Polyhedron parseObj(string filename);
private:
    vector<Vector3f> vertices;
    vector<Vector3f> normals;
};

#endif
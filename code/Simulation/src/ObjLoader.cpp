// inspired by: http://weblog.benjaminsommer.com/blog/2012/02/12/a-tiny-wavefront-object-loader-part1/
#include <iostream>
#include <fstream>
#include <sstream>

#include "ObjLoader.h"

#include "Polyhedron.h"
#include "Face.h"

using namespace std;

ObjLoader::ObjLoader() {}
ObjLoader::~ObjLoader() {}

Polyhedron ObjLoader::parseObj(std::string filename)
{
    std::cout << "parsing: " << filename << std::endl;

    Polyhedron polyhedron;
    string line;
    string key;

    this->vertices.clear();
    this->normals.clear();

    ifstream objFile (filename.c_str());
    if (objFile.is_open())
    {
        while (objFile.good() && !objFile.eof() && getline(objFile, line))
        {
            key = "";
            stringstream str(line);
            str >> key >> ws;
            if (key == "v")
            {
                float x, y, z;
                str >> x >> ws >> y >> ws >> z;
                polyhedron.vertices.push_back(x);
                polyhedron.vertices.push_back(y);
                polyhedron.vertices.push_back(z);

                Vector3f v(x, y, z);
                this->vertices.push_back(v);
            }
            else if (key == "vn")
            {
                float x, y, z;
                str >> x >> ws >> y >> ws >> z;
                Vector3f n(x, y, z);
                this->normals.push_back(n);
            }
            else if (key == "f")
            {
                Face f;
                unsigned int vertex_index, texture_index, normal_index;
                while (!str.eof())
                {
                    str >> vertex_index;
                    vertex_index--;

                    polyhedron.indices.push_back(vertex_index);
                    f.vertices.push_back(this->vertices.at(vertex_index));

                    if (str.get() == '/')
                    {
                        if (str.peek() != '/')
                        {
                            str >> texture_index;
                            texture_index--;
                        }
                        if (str.get() == '/')
                        {
                            str >> normal_index;
                            normal_index--;

                            Vector3f normal = this->normals.at(normal_index);
                            polyhedron.normals.push_back(normal[0]);
                            polyhedron.normals.push_back(normal[1]);
                            polyhedron.normals.push_back(normal[2]);

                            f.normal = normal;
                        }
                    }
                }
                polyhedron.faces.push_back(f);
            }
        }
        objFile.close();
    }
    else
    {
        std::cout << "Unable to open file" << std::endl;
    }

    polyhedron.computeProperties();

    return polyhedron;
}
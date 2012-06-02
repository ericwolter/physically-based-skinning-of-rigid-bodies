// inspired by: http://weblog.benjaminsommer.com/blog/2012/02/12/a-tiny-wavefront-object-loader-part1/

#include "ObjLoader.h"

#include "Polyhedron.h"
#include "Face.h"

#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

ObjLoader::ObjLoader() {}
ObjLoader::~ObjLoader() {}

void ObjLoader::parseObj(std::string filename) {
	std::cout << "parsing: " << filename << std::endl;

	Polyhedron polyhedron;
	string line;
	string key;

	this->vertices.clear();
	this->normals.clear();

	ifstream objFile (filename.c_str());
	if(objFile.is_open()) {
		while (objFile.good() && !objFile.eof() && getline(objFile, line)) {
			key = "";
    		stringstream str(line);
    		str >> key >> ws;
    		if (key == "v") {
    			double x, y, z;
    			str >> x >> ws >> y >> ws >> z;
    			Vector3d v(x,y,z);
    			this->vertices.push_back(v);
    		} else if (key == "vn") {
    			double x, y, z;
    			str >> x >> ws >> y >> ws >> z;
    			Vector3d n(x,y,z);
    			this->normals.push_back(n);
    		} else if (key == "f") {
    			Face f;
				int vertex_index, texture_index, normal_index;
				while(!str.eof()) {
    				str >> vertex_index;
    				vertex_index--;
        			f.vertices.push_back(this->vertices.at(vertex_index));
    				if (str.get() == '/') {
    					if (str.peek() != '/') {
    						str >> texture_index;
    						texture_index--;
    					}
    					if (str.get() == '/') {
    						str >> normal_index;
    						normal_index--;
    						f.normal = this->normals.at(normal_index);
    					}
    				}
				}
                polyhedron.faces.push_back(f);
    		}
    	}
    	objFile.close();
	} else {
		std::cout << "Unable to open file" << std::endl;
	}

	polyhedron.computeProperties();
}
#include "ObjLoader.h"

#include <iostream>
#include <fstream>
#include <sstream>

void ObjLoader::parseObj(std::string filename) {
	std::string line;
	std::ifstream objFile (filename.c_str());
	if(objFile.is_open()) {
		while(objFile.good()) {
			std::getline (objFile, line);
			parseLine(line);
		}
		objFile.close();
	} else {
		std::cout << "Unable to open file";
	}
}

bool ObjLoader::parseLine(std::string line) {
	std::string obj_operator;

	if (line.empty()) {
		return true;
	} else {
		std::stringstream ss (std::stringstream::in | std::stringstream::out);
		ss.str(line);
		ss >> obj_operator;
		if (obj_operator.compare("#") == 0) {
			return true;
		} else if (obj_operator.compare("v") == 0) {
			double x, y, z;
			ss >>x >>y >>z;
		}
		if (ss.fail()) {
			return false;
		}
		return true;
	}
}
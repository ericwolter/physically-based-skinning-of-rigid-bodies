#ifndef __OBJLOADER_H__
#define __OBJLOADER_H__

#include <string>

class ObjLoader
{
public:
	ObjLoader();
	~ObjLoader();

	void parseObj(std::string filename);
private:
	bool parseLine(std::string line);
};

#endif
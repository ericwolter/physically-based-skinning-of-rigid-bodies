//
//  ObjLoader.cpp
//  BachelorThesis
//
//  Created by Eric Wolter on 9/22/12.
//  Copyright (c) 2012 Eric Wolter. All rights reserved.
//

#include "ObjLoader.h"

// inspirhttp://weblog.benjaminsommer.com/blog/2012/02/12/a-tiny-wavefront-object-loader-part1/ed by: 
#include <iostream>
#include <fstream>
#include <sstream>

#include "ObjLoader.h"

#include "ViewModel.h"

using namespace std;

ObjLoader::ObjLoader() {}
ObjLoader::~ObjLoader() {}

ViewModel ObjLoader::parseObj(std::string filename)
{
    ViewModel viewModel;
    string line;
    string key;
    
    vector<Vector3f> vertices;
    vector<Vector3f> normals;

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
                viewModel.vertices.push_back(x);
                viewModel.vertices.push_back(y);
                viewModel.vertices.push_back(z);
                
                Vector3f v(x, y, z);
                vertices.push_back(v);
            }
            else if (key == "vn")
            {
                float x, y, z;
                str >> x >> ws >> y >> ws >> z;
                Vector3f n(x, y, z);
                normals.push_back(n);
            }
            else if (key == "f")
            {
                unsigned int vertex_index, texture_index, normal_index;
                while (!str.eof())
                {
                    str >> vertex_index;
                    vertex_index--;
                    
                    viewModel.indices.push_back(vertex_index);
                    
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
                            
                            Vector3f normal = normals.at(normal_index);
                            viewModel.normals.push_back(normal[0]);
                            viewModel.normals.push_back(normal[1]);
                            viewModel.normals.push_back(normal[2]);
                        }
                    }
                }
            }

        }
        objFile.close();
    }
    else
    {
        std::cout << "Unable to open file" << std::endl;
    }
    
    return viewModel;
}

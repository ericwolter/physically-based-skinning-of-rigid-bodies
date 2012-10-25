//
//  ObjLoader.h
//  BachelorThesis
//
//  Created by Eric Wolter on 9/22/12.
//  Copyright (c) 2012 Eric Wolter. All rights reserved.
//

#ifndef __BachelorThesis__ObjLoader__
#define __BachelorThesis__ObjLoader__

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

class ViewModel;

class ObjLoader
{
public:
    ObjLoader();
    ~ObjLoader();
    
    static ViewModel parseObj(string filename);
};

#endif /* defined(__BachelorThesis__ObjLoader__) */

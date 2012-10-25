//
//  ViewModel.h
//  BachelorThesis
//
//  Created by Eric Wolter on 9/22/12.
//  Copyright (c) 2012 Eric Wolter. All rights reserved.
//

#ifndef __BachelorThesis__ViewModel__
#define __BachelorThesis__ViewModel__

#define GLFW_INCLUDE_GL3
#define GLFW_NO_GLU
#include <GL/glfw.h>

#include <iostream>
#include <vector>
using namespace std;

#include <Eigen/Dense>
using namespace Eigen;

class ViewModel
{
public:
    ViewModel();
    ~ViewModel();
    
    vector<float> vertices;
    vector<unsigned int> indices;
    vector<float> normals;
    
    void init(Vector4f color);
    void render(Affine3f cameraTransform, Vector3f position, Quaternionf orientation, GLuint modelToCameraMatrixUnif);
    void render(Affine3f cameraTransform, Vector3f position, Quaternionf orientation, Vector3f scale, GLuint modelToCameraMatrixUnif);
private:
    GLuint vertexArrayObject;
    GLuint vertexBufferObject;
    GLuint colorBufferObject;
    GLuint indexBufferObject;
    GLuint normalBufferObject;
    
    Vector3f centerOfMass;
};


#endif /* defined(__BachelorThesis__ViewModel__) */

//
//  Simulation.h
//  BachelorThesis
//
//  Created by Eric Wolter on 9/23/12.
//  Copyright (c) 2012 Eric Wolter. All rights reserved.
//

#ifndef __BachelorThesis__Simulation__
#define __BachelorThesis__Simulation__

#include <iostream>
#include <vector>
using namespace std;

#define GLFW_INCLUDE_GL3
#define GLFW_NO_GLU
#include <GL/glfw.h>

#include <Eigen/Dense>
using namespace Eigen;

#include <BulletDynamics/btBulletDynamicsCommon.h>

#include "ViewModel.h"
#include "CombinedBody.h"

class Simulation
{
public:
    Simulation();
    ~Simulation();
    
    void init();
    void step(float timeStep);
    void render(Affine3f cameraTransform, GLuint modelToCameraMatrixUnif);
    
    vector<CombinedBody> bodies;
private:
    void predictUnconstrainedMotion(float timeStep);
    void integrateTransforms(float timeStep);
    
    ViewModel groundViewModel;
    ViewModel cubeViewModel;
    ViewModel cubeViewModel2;
    ViewModel sphereViewModel;
    
    btDefaultCollisionConfiguration *collisionConfiguration;
    btCollisionDispatcher* dispatcher;
    btDispatcherInfo dispatchInfo;
};

#endif /* defined(__BachelorThesis__Simulation__) */

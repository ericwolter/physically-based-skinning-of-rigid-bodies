//
//  Simulation.cpp
//  BachelorThesis
//
//  Created by Eric Wolter on 9/23/12.
//  Copyright (c) 2012 Eric Wolter. All rights reserved.
//

// adapted from: https://code.google.com/p/bullet/source/browse/trunk/src/BulletDynamics/Dynamics/btSimpleDynamicsWorld.cpp
#include "Simulation.h"

#include "ObjLoader.h"

#include <BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h>
#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"

Simulation::Simulation()
{
}

Simulation::~Simulation()
{
}

void Simulation::init()
{
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    
    Vector4f bodyColor1;
    bodyColor1[0] = 220.0f / 255.0f;
    bodyColor1[1] = 50.0f / 255.0f;
    bodyColor1[2] = 40.0f / 255.0f;
    bodyColor1[3] = 1.0f;
    
    Vector4f particleColor;
    particleColor[0] = 50.0f / 255.0f;
    particleColor[1] = 220.0f / 255.0f;
    particleColor[2] = 40.0f / 255.0f;
    particleColor[3] = 1.0f;
    
    Vector4f groundColor;
    groundColor[0] = 0.0f / 255.0f;
    groundColor[1] = 40.0f / 255.0f;
    groundColor[2] = 50.0f / 255.0f;
    groundColor[3] = 1.0f;
    
    groundViewModel = ObjLoader::parseObj("plane.model");
    groundViewModel.init(groundColor);
    
    cubeViewModel = ObjLoader::parseObj("cube.model");
    cubeViewModel.init(bodyColor1);
    
    cubeViewModel2 = ObjLoader::parseObj("cube.model");
    cubeViewModel2.init(bodyColor1);
    
    sphereViewModel = ObjLoader::parseObj("sphere.model");
    sphereViewModel.init(particleColor);
    
    btCollisionShape *groundShape = new btBoxShape(btVector3(btScalar(50.0f), btScalar(0.5f), btScalar(50.0f)));
    
    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0,0,0));
    
    btScalar groundMass(0.);
    btVector3 groundLocalInertia(0,0,0);
    
    btDefaultMotionState* groundMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo groundInfo(groundMass, groundMotionState, groundShape, groundLocalInertia);
    btRigidBody *groundBody = new btRigidBody(groundInfo);
    
    btCollisionShape *cubeShape = new btBoxShape(btVector3(btScalar(1.0f), btScalar(1.0f), btScalar(1.0f)));
    
    btTransform cubeTransform;
    cubeTransform.setIdentity();
    cubeTransform.setOrigin(btVector3(0,10,0));
//    cubeTransform.setRotation(btQuaternion(btVector3(0,0,1), btScalar(0.52)));
    
    btScalar cubeMass(1.0f);
    btVector3 cubeLocalInertia(0,0,0);
    
    cubeShape->calculateLocalInertia(cubeMass, cubeLocalInertia);
    btDefaultMotionState* cubeMotionState = new btDefaultMotionState(cubeTransform);
    btRigidBody::btRigidBodyConstructionInfo cubeInfo(cubeMass, cubeMotionState, cubeShape, cubeLocalInertia);
    btRigidBody *cubeBody = new btRigidBody(cubeInfo);
    
    cubeBody->setGravity(btVector3(0,-10,0));
    
//    btCollisionShape *cubeShape2 = new btBoxShape(btVector3(btScalar(1.0f), btScalar(1.0f), btScalar(1.0f)));
//    
//    btTransform cubeTransform2;
//    cubeTransform2.setIdentity();
//    cubeTransform2.setOrigin(btVector3(0,5,0));
//    cubeTransform2.setRotation(btQuaternion(btVector3(0,0,1), btScalar(0.13)));
//    
//    btScalar cubeMass2(1.0f);
//    btVector3 cubeLocalInertia2(0,0,0);
//    
//    cubeShape->calculateLocalInertia(cubeMass2, cubeLocalInertia2);
//    btDefaultMotionState* cubeMotionState2 = new btDefaultMotionState(cubeTransform2);
//    btRigidBody::btRigidBodyConstructionInfo cubeInfo2(cubeMass2, cubeMotionState2, cubeShape2, cubeLocalInertia2);
//    btRigidBody *cubeBody2 = new btRigidBody(cubeInfo2);
//    
//    cubeBody2->setGravity(btVector3(0,-10,0));
    
    CombinedBody *groundCombinedBody = new CombinedBody(groundBody);
    CombinedBody *cubeCombinedBody = new CombinedBody(cubeBody);
//    CombinedBody *cubeCombinedBody2 = new CombinedBody(cubeBody2);
    
    bodies.push_back(*groundCombinedBody);
    bodies.push_back(*cubeCombinedBody);
//    bodies.push_back(*cubeCombinedBody2);
}

void Simulation::render(Affine3f cameraTransform, GLuint modelToCameraMatrixUnif)
{
    btVector3 position;
    btTransform transform;
    btQuaternion quaternion;
    btVector3 axis;
    btScalar angle;
    
    Vector3f p;
    Quaternionf q;
    
    btRigidBody *groundBody = bodies[0].m_rigidBody;
    btRigidBody *cubeBody = bodies[1].m_rigidBody;
//    btRigidBody *cubeBody2 = bodies[2].m_rigidBody;
    
    position = groundBody->getCenterOfMassPosition();
    p = Vector3f(position.x(), position.y(), position.z());
    
    transform = groundBody->getCenterOfMassTransform();
    quaternion = transform.getRotation();
    axis = quaternion.getAxis();
    angle = quaternion.getAngle();
    q = AngleAxis<float>(angle, Vector3f(axis.x(), axis.y(), axis.z()));
    
    groundViewModel.render(cameraTransform, p, q, modelToCameraMatrixUnif);
    
    position = cubeBody->getCenterOfMassPosition();
    p = Vector3f(position.x(), position.y(), position.z());
    
    transform = cubeBody->getCenterOfMassTransform();
    quaternion = transform.getRotation();
    axis = quaternion.getAxis();
    angle = quaternion.getAngle();
    q = AngleAxis<float>(angle, Vector3f(axis.x(), axis.y(), axis.z()));
    
    cubeViewModel.render(cameraTransform, p, q, modelToCameraMatrixUnif);
    
    vector<Particle*>::iterator pIter;
    for(pIter=bodies[1].m_particles.begin(); pIter != bodies[1].m_particles.end(); pIter++)
    {
        Particle *p = *pIter;
        sphereViewModel.render(cameraTransform, p->position, p->orientation, p->radii, modelToCameraMatrixUnif);
        cout << "p.position: " << p->position << endl;
    }
}

static btVoronoiSimplexSolver sGjkSimplexSolver;
btSimplexSolverInterface& gGjkSimplexSolver = sGjkSimplexSolver;

void Simulation::step(float timeStep)
{
    predictUnconstrainedMotion(timeStep);
    
    for (int b1 = 0; b1 < bodies.size(); b1++)
    {
        for(int b2 = b1 + 1; b2 < bodies.size(); b2++)
        {
            CombinedBody *c1 = &bodies[b1];
            CombinedBody *c2 = &bodies[b2];
            
            if (!c1->m_rigidBody->isActive() || c1->m_rigidBody->isStaticObject())
            {
                if (!c2->m_rigidBody->isActive() || c2->m_rigidBody->isStaticObject())
                {
                    // two rigid bodies, should not happen...
                }
                else
                {
                    // c1 one is plane and c2 is a combined body
                    // find all collision points
                    // find closest plane equation
                    
                    for(int i=0; i< c2->m_outerParticles.size(); i++)
                    {
                        Particle *p = c2->m_outerParticles.at(i);
                        
                        Matrix3f radiiMatrix;
                        radiiMatrix << p->radii.x() * p->radii.x(),  0,                      0,
                        0,                      p->radii.y() * p->radii.y(),  0,
                        0,                      0,                      p->radii.z() * p->radii.z();
                        
                        Matrix3f AInv = p->predictedOrientation.toRotationMatrix() * radiiMatrix * p->predictedOrientation.toRotationMatrix().transpose();
                        
                        btBoxShape *planeShape = (btBoxShape *)c1->m_rigidBody->getCollisionShape();
                        // only interested in y plane
                        btVector3 n2 = btVector3(0,1,0);
                        btVector3 p2 = planeShape->localGetSupportingVertex(n2);
                            
                        Vector3f n1 = Vector3f(n2.x(), n2.y(), n2.z());
                        Vector3f p1 = Vector3f(p2.x(), p2.y(), p2.z());
                        float d = n1.transpose() * p1;
//                        cout << "plane_n: " << n1 << endl;
//                        cout << "plane_p: " << p1 << endl;
//                        cout << "plane_d: " << d << endl;
                        
                        float test = n1.transpose() * AInv * n1;
                        
                        Vector3f x1 = (AInv * n1)/sqrt(test);
                        Vector3f x2 = -x1;
                        x1 = x1+p->predictedPosition;
                        x2 = x2+p->predictedPosition;
                        
                        float d1 = n1.dot(x1);
                        float d2 = n1.dot(x2);
                        
                        float smallerD;
                        if(d1<d2) {
                            smallerD = d1;
                        } else {
                            smallerD = d2;
                        }
                            
                        if(smallerD < d) {
                            cout << "collision: " << endl;
                        }
                    }
                }
            }
            else if (!c2->m_rigidBody->isActive() || c2->m_rigidBody->isStaticObject())
            {
                // c2 is plane and c1 is combined body
            }
            else
            {
                // both c1 and c2 are combined bodies
            }
        }
    }
    
    for(int i = 0; i < 4; i++)
    {
        for (int b1 = 0; b1 < bodies.size(); b1++)
        {
            CombinedBody *c1 = &bodies[b1];
            
            if (c1->m_rigidBody->isActive() && !c1->m_rigidBody->isStaticObject())
            {
                // update all implicit shape matching groups
                vector<ParticleGroup*>::iterator group;
                for (group = c1->m_particleGroups.begin(); group != c1->m_particleGroups.end(); group++)
                {
                    ParticleGroup *g = *group;
                    g->update();
                }
                
                // calculate forces for resulting from attached particles
                vector<Particle*>::iterator particle;
                for (particle = c1->m_attachedParticles.begin(); particle != c1->m_attachedParticles.end(); particle++)
                {
                    Particle *p = *particle;
                    btVector3 tmpBodyPosition = c1->m_rigidBody->getInterpolationWorldTransform().getOrigin();
                    Vector3f bodyPosition = Vector3f(tmpBodyPosition.x(), tmpBodyPosition.y(), tmpBodyPosition.z());
                    
                    Vector3f fixedPosition = bodyPosition + p->relativePosition;
                    Vector3f diffVector = fixedPosition - p->predictedPosition;
                    p->predictedPosition = fixedPosition;
                }
            }
        }
        
    }
    
//    btManifoldArray manifoldArray;
//    for (int b1 = 0; b1 < bodies.size(); b1++)
//    {
//        for(int b2 = b1 + 1; b2 < bodies.size(); b2++)
//        {
//            // collision detection
//            btCollisionAlgorithm *algo = dispatcher->findAlgorithm(bodies[b1].m_rigidBody, bodies[b2].m_rigidBody);
//            btManifoldResult contactPointResult(bodies[b1].m_rigidBody,bodies[b2].m_rigidBody);
//            dispatchInfo.m_timeStep = timeStep;
//            dispatchInfo.m_stepCount = 0;
//            
//            algo->processCollision(bodies[b1].m_rigidBody, bodies[b2].m_rigidBody, dispatchInfo, &contactPointResult);
//            btManifoldArray m;
//            algo->getAllContactManifolds(m);
//            
//            for (int i=0; i < m.size(); i++) {
//                manifoldArray.push_back(m.at(i));
//            }
//        }
//    }
//            
//    // collision resolution
//    for(int s = 0; s < 4; s++)
//    {
//        int numManifolds = manifoldArray.size();
//        cout << "numManifolds: " << numManifolds << endl;
//        for(int i = 0; i < numManifolds; i++)
//        {
//            btPersistentManifold *contactManifold = manifoldArray[i];
//            btRigidBody *body1 = (btRigidBody *)contactManifold->getBody0();
//            btRigidBody *body2 = (btRigidBody *)contactManifold->getBody1();
//            
//            int numContacts = contactManifold->getNumContacts();
//            cout << "numContacts: " << numContacts << endl;
//            if(numContacts > 0)
//            {
//                for(int c = 0; c < numContacts; c++)
//                {
//                    btManifoldPoint contact = contactManifold->getContactPoint(c);
//                    
//                    btVector3 hitPoint = (contact.getPositionWorldOnA() + contact.getPositionWorldOnB()) / 2;
//                    btVector3 normal = contact.m_normalWorldOnB;
//                    btScalar penDepth = fabs(contact.getDistance());
//                    
//                    btVector3 r1 = hitPoint - body1->getCenterOfMassPosition();
//                    btVector3 r2 = hitPoint - body2->getCenterOfMassPosition();
//                    
//                    btVector3 v1 = body1->getLinearVelocity() + (body1->getAngularVelocity().cross(r1));
//                    btVector3 v2 = body2->getLinearVelocity() + (body2->getAngularVelocity().cross(r2));
//                    
//                    btScalar vrel = normal.dot(v1 - v2);
//                    
//                    // src: http://www.xbdev.net/physics/RigidBodyImpulseCubes/index.php
//                    btScalar allowedPenetration = 0.1f;
//                    btScalar biasFactor = 0.1;
//                    btScalar bias;
//                    if ((penDepth - allowedPenetration) < 0) {
//                        bias = 0;
//                    }
//                    else
//                    {
//                        bias = biasFactor * (1/timeStep) * (penDepth - allowedPenetration);
//                    }
//                    
//                    btScalar numerator = (-vrel + bias);
//                    btScalar term1 = body1->getInvMass();
//                    btScalar term2 = body2->getInvMass();
//                    btScalar term3 = normal.dot((body1->getInvInertiaTensorWorld() * r1.cross(normal)).cross(r1));
//                    btScalar term4 = normal.dot((body2->getInvInertiaTensorWorld() * r2.cross(normal)).cross(r2));
//                    
//                    btScalar j = numerator / (term1 + term2 + term3 + term4);
//                    if (j < 0)
//                    {
//                        j = 0;
//                    }
//                    btVector3 force = j * normal;
//                    
//                    body1->setLinearVelocity(body1->getLinearVelocity() + body1->getInvMass() * force);
//                    body1->setAngularVelocity(body1->getAngularVelocity() + r1.cross(force) * body1->getInvInertiaTensorWorld());
//                    body2->setLinearVelocity(body2->getLinearVelocity() - body2->getInvMass() * force);
//                    body2->setAngularVelocity(body2->getAngularVelocity() - r2.cross(force) * body2->getInvInertiaTensorWorld());
//                }
//            }
//        }
//    }

    // integration
    integrateTransforms(timeStep);
}

void Simulation::predictUnconstrainedMotion(float timeStep)
{
    vector<CombinedBody>::iterator body;
    for (body = bodies.begin(); body != bodies.end(); body++)
    {
        btRigidBody *rigid = body->m_rigidBody;
        if (rigid->isActive() && (!rigid->isStaticObject()))
        {
            body->applyGravity(timeStep);
            body->integrateVelocities(timeStep);
            body->predictIntegratedTransform(timeStep, rigid->getInterpolationWorldTransform());
        }
    }
}

void Simulation::integrateTransforms(float timeStep)
{
    vector<CombinedBody>::iterator body;
    for (body = bodies.begin(); body != bodies.end(); body++)
    {
        body->integrateTransforms(timeStep);
    }
}
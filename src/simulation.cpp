#include <iostream>
using namespace std;

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include <GL/glut.h>
#endif

#include <BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h>
#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>

#include "constants.h"
#include "simulation.h"

Simulation::Simulation() {}
Simulation::~Simulation() {}

void Simulation::init(bool useDeformable) {
	this->useDeformable = useDeformable;
	
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    
    btVector3 gravity = btVector3(0,-10,0);
    
	// setup ground
    btCollisionShape *groundShape = new btBoxShape(btVector3(btScalar(50.0f), btScalar(0.5f), btScalar(50.0f)));
    
    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0,0,0));
    
    btScalar groundMass(0.);
    btVector3 groundLocalInertia(0,0,0);
    
    btDefaultMotionState* groundMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo groundInfo(groundMass, groundMotionState, groundShape, groundLocalInertia);
	
    ground = new btRigidBody(groundInfo);
    
    // setup block
    btCollisionShape *blockShape = new btBoxShape(btVector3(1.5f, 1.0f, 1.0f));
    
    btTransform blockTransform;
    blockTransform.setIdentity();
    blockTransform.setOrigin(btVector3(0.0f,1.5f,0.0f));
    
    btScalar blockMass(1.0f);
    btVector3 blockLocalInertia(0,0,0);
    
    blockShape->calculateLocalInertia(blockMass, blockLocalInertia);
    btDefaultMotionState *blockMotionState = new btDefaultMotionState(blockTransform);
    btRigidBody::btRigidBodyConstructionInfo blockInfo(blockMass, blockMotionState, blockShape, blockLocalInertia);
    
    block = new btRigidBody(blockInfo);
    block->setGravity(gravity);
    
    if (useDeformable) {
        // setup test cube
        btCollisionShape *testCubeShape = new btBoxShape(btVector3(1.0f, 1.0f, 1.0f));
        
        btTransform testCubeTransform;
        testCubeTransform.setIdentity();
        testCubeTransform.setOrigin(btVector3(0,5,0));
        
        btScalar testCubeMass(1.0f);
        btVector3 testCubeLocalInertia(0,0,0);
        
        testCubeShape->calculateLocalInertia(testCubeMass, testCubeLocalInertia);
        btDefaultMotionState* testCubeMotionState = new btDefaultMotionState(testCubeTransform);
        btRigidBody::btRigidBodyConstructionInfo testCubeInfo(testCubeMass, testCubeMotionState, testCubeShape, testCubeLocalInertia);
        
        btRigidBody *testCube = new btRigidBody(testCubeInfo);
        testCube->setGravity(gravity);
        
        softCube = new CombinedBody(testCube);
    } else {
        // setup finger 1
        btCollisionShape *finger1Shape = new btBoxShape(btVector3(1.0f, 1.0f, 3.0f));
        
        btTransform finger1Transform;
        finger1Transform.setIdentity();
        finger1Transform.setOrigin(btVector3(-2.1f,3,0.0f));
        finger1Transform.setRotation(btQuaternion(btVector3(0,1,0), 0.2));
        
        btScalar finger1Mass(10.0f);
        btVector3 finger1LocalInertia(0,0,0);
        
        finger1Shape->calculateLocalInertia(finger1Mass, finger1LocalInertia);
        btDefaultMotionState* finger1MotionState = new btDefaultMotionState(finger1Transform);
        btRigidBody::btRigidBodyConstructionInfo finger1Info(finger1Mass, finger1MotionState, finger1Shape, finger1LocalInertia);
        
        finger1 = new btRigidBody(finger1Info);
        finger1->setGravity(gravity);
        
        // setup finger 2
        btCollisionShape *finger2Shape = new btBoxShape(btVector3(1.0f, 1.0f, 3.0f));
        
        btTransform finger2Transform;
        finger2Transform.setIdentity();
        finger2Transform.setOrigin(btVector3(2.1f,3,0.0f));
        finger2Transform.setRotation(btQuaternion(btVector3(0,1,0), -0.2));
        
        btScalar finger2Mass(10.0f);
        btVector3 finger2LocalInertia(0,0,0);
        
        finger2Shape->calculateLocalInertia(finger2Mass, finger2LocalInertia);
        btDefaultMotionState* finger2MotionState = new btDefaultMotionState(finger2Transform);
        btRigidBody::btRigidBodyConstructionInfo finger2Info(finger2Mass, finger2MotionState, finger2Shape, finger2LocalInertia);
        
        finger2 = new btRigidBody(finger2Info);
        finger2->setGravity(gravity);
    }
}

static btVoronoiSimplexSolver sGjkSimplexSolver;
btSimplexSolverInterface& gGjkSimplexSolver = sGjkSimplexSolver;

void Simulation::step(float timeStep) {
	predictUnconstrainedMotion(timeStep);
	
    if(useDeformable) {
        // collision
        for(int i=0; i< softCube->m_outerParticles.size(); i++)
        {
            Particle *particle = softCube->m_outerParticles.at(i);
            particle->hasCollided = false;
            
            btMatrix3x3 radiiMatrix = btMatrix3x3(particle->radii.x() * particle->radii.x(), 0, 0, 0, particle->radii.y() * particle->radii.y(), 0, 0, 0, particle->radii.z() * particle->radii.z());
            
            btMatrix3x3 R = btMatrix3x3(particle->predictedOrientation);
            btMatrix3x3 AInv = R * radiiMatrix * R.transpose();
            
            // collision with ground
            btBoxShape *planeShape = (btBoxShape *)ground->getCollisionShape();
            // only interested in y plane
            btVector3 n = btVector3(0,1,0);
            btVector3 p = planeShape->localGetSupportingVertex(n);
            
            btScalar d = n.dot(p);
            
            btVector3 tmp1 = n * AInv;
            float tmp2 = tmp1.dot(n);
            float test = tmp2;
            
            btVector3 x1 = (AInv * n)/sqrt(test);
            btVector3 x2 = -x1;
            x1 = x1+particle->predictedPosition;
            x2 = x2+particle->predictedPosition;
            
            float d1 = n.dot(x1);
            float d2 = n.dot(x2);
            
            float smallerD;
            btVector3 smallerX;
            if(d1<d2) {
                smallerD = d1;
                smallerX = x1;
            } else {
                smallerD = d2;
                smallerX = x2;
            }
            
            if(smallerD < d) {
                particle->predictedPosition += n * (d - smallerD);
                particle->hasCollided = true;
                particle->surfaceNormal = n;
                particle->surfaceVelocity = btVector3(0,0,0);
            }
            
            // collision wiht block
            btBoxShape *blockShape = (btBoxShape *)block->getCollisionShape();
            int numPlanes = blockShape->getNumPlanes();
            for(int pI =0; pI < numPlanes; pI++) {
                btVector4 plane;
                blockShape->getPlaneEquation(plane, pI);
                
                n = btVector3(plane.x(), plane.y(), plane.z());
                d = -plane.w();
                p = d * n;
                
                // transform to world space
                btTransform t = block->getInterpolationWorldTransform();
                p = t * p;
                n = t.getBasis() * n;
                n = n.normalized();
                d = n.dot(p);
                
                btVector3 tmp1 = n * AInv;
                float tmp2 = tmp1.dot(n);
                float test = tmp2;
                
                btVector3 x1 = (AInv * n)/sqrt(test);
                btVector3 x2 = -x1;
                x1 = x1+particle->predictedPosition;
                x2 = x2+particle->predictedPosition;
                
                // transform to body
                btVector3 bodyX1 = t.inverse() * x1;
                btVector3 bodyX2 = t.inverse() * x2;
                

                float d1 = n.dot(x1);
                float d2 = n.dot(x2);
                
                if(d1 < d2) {
                    if(blockShape->isInside(bodyX1, 0.00001f)) {
                        cout << "inside1" << endl;
//                        particle->hasCollided = true;
//                        particle->predictedPosition += n * (d - d1);
                    }
                } else {
                    if (blockShape->isInside(bodyX2, 0.00001f)) {
                        cout << "inside2" << endl;
                        particle->hasCollided = true;
                        particle->predictedPosition += n * (d - d2);
                    }
                }
            }
        }
        
        // gauss seidel
        for (int i = 0; i < 4; i++) {
            
            // shape matching
            vector<ParticleGroup*>::iterator g;
            for(g = softCube->m_particleGroups.begin(); g != softCube->m_particleGroups.end(); g++) {
                (*g)->update();
            }
        }
        
        for(int i = 0; i < softCube->m_outerParticles.size(); i++) {
            Particle *particle = softCube->m_outerParticles.at(i);
            if(particle->hasCollided) {
//                const float s_linear = 0.5f;
//                const float s_angular = 0.5f;
//                
//                //friction
//                btVector3 dv = particle->surfaceVelocity - particle->linearVelocity;
//                
//                //linear
//                btVector3 pv = dv.dot(particle->surfaceNormal) * particle->surfaceNormal;
//                btVector3 tv = dv - pv;
//                
//                btVector3 linearFriction = tv * s_linear;
//                //angular
//                btVector3 r = particle->radii * particle->surfaceNormal;
//                dv = dv - particle->angularVelocity.cross(r);
//                
//                btVector3 test1 = r/(r.length2());
//                btVector3 test2 = test1.cross(dv);
//                
//                btVector3 angularFriction = test2 * s_angular;
//                
//                particle->linearVelocity += linearFriction;
//                particle->angularVelocity += angularFriction;
//                
//                particle->hasCollided = false;
            }
        }
        
        //
        for(int i = 0; i < softCube->m_attachedParticles.size(); i++) {
            Particle *particle = softCube->m_attachedParticles.at(i);
            
            btTransform bodyTransform = softCube->m_rigidBody->getInterpolationWorldTransform();
            btVector3 truePosition = bodyTransform * particle->relativePosition;
            
            btVector3 particlePosition = particle->predictedPosition;
            btVector3 delta = particlePosition - truePosition;
            btVector3 impulse = (particle->mass * delta) / timeStep;
            
            btVector3 bodyVelocity = softCube->m_rigidBody->getLinearVelocity();
            softCube->m_rigidBody->applyImpulse(impulse, particle->relativePosition);
            bodyVelocity = softCube->m_rigidBody->getLinearVelocity();
            particle->predictedPosition = truePosition;
        }
    }
	collision(timeStep);
	
	integrateTransforms(timeStep);
    
    ground->clearForces();
    block->clearForces();
    
    if(useDeformable) {
        softCube->m_rigidBody->clearForces();
    } else {
        finger1->clearForces();
        finger2->clearForces();
        block->clearForces();
    }
}

void Simulation::collision(float timeStep) {

    int numBodies = 0;
    btRigidBody** bodies;
    if(useDeformable) {
        numBodies = 2;
        btRigidBody* tmp[2] = {block, ground};
        bodies = tmp;
    } else {
        numBodies = 4;
        btRigidBody* tmp[4] = {block, ground, finger1, finger2};
        bodies = tmp;
    }
    btManifoldArray manifoldArray;
   
    for(int b1 = 0; b1 < numBodies; b1++) {
        for(int b2 = b1+1 ; b2 < numBodies; b2++) {
            btCollisionAlgorithm *algo = dispatcher->findAlgorithm(bodies[b1], bodies[b2]);
            btManifoldResult contactPointResult(bodies[b1],bodies[b2]);
            dispatchInfo.m_timeStep = timeStep;
            dispatchInfo.m_stepCount = 0;
           
            algo->processCollision(bodies[b1], bodies[b2], dispatchInfo, &contactPointResult);
            btManifoldArray m;
            algo->getAllContactManifolds(m);
           
            for (int i=0; i < m.size(); i++) {
               manifoldArray.push_back(m.at(i));
            }
        }
    }
           
    for(int s = 0; s < 4; s++)
    {
       int numManifolds = manifoldArray.size();
       for(int i = 0; i < numManifolds; i++)
       {
           btPersistentManifold *contactManifold = manifoldArray[i];
           btRigidBody *body1 = (btRigidBody *)contactManifold->getBody0();
           btRigidBody *body2 = (btRigidBody *)contactManifold->getBody1();
           
           int numContacts = contactManifold->getNumContacts();
           if(numContacts > 0)
           {
               for(int c = 0; c < numContacts; c++)
               {
                   btManifoldPoint contact = contactManifold->getContactPoint(c);
                   
                   btVector3 hitPoint = (contact.getPositionWorldOnA() + contact.getPositionWorldOnB()) / 2;
                   btVector3 normal = contact.m_normalWorldOnB;
                   btScalar penDepth = fabs(contact.getDistance());
                   
                   btVector3 r1 = hitPoint - body1->getCenterOfMassPosition();
                   btVector3 r2 = hitPoint - body2->getCenterOfMassPosition();
                   
                   btVector3 v1 = body1->getLinearVelocity() + (body1->getAngularVelocity().cross(r1));
                   btVector3 v2 = body2->getLinearVelocity() + (body2->getAngularVelocity().cross(r2));
                   
                   btScalar vrel = normal.dot(v1 - v2);
                   
                   // src: http://www.xbdev.net/physics/RigidBodyImpulseCubes/index.php
                   // normal force
                   btScalar allowedPenetration = 0.1f;
                   btScalar biasFactor = 0.3f;
                   btScalar bias;
                   if ((penDepth - allowedPenetration) < 0.0f) {
                       bias = 0;
                   }
                   else
                   {
                       bias = biasFactor * (1/timeStep) * (penDepth - allowedPenetration);
                   }
                   
                   btScalar normal_numerator = (-vrel + bias);
                   btScalar normal_term1 = body1->getInvMass();
                   btScalar normal_term2 = body2->getInvMass();
                   btScalar normal_term3 = normal.dot((body1->getInvInertiaTensorWorld() * r1.cross(normal)).cross(r1));
                   btScalar normal_term4 = normal.dot((body2->getInvInertiaTensorWorld() * r2.cross(normal)).cross(r2));
                   
                   btScalar normal_j = normal_numerator / (normal_term1 + normal_term2 + normal_term3 + normal_term4);
                   if (normal_j < 0)
                   {
                       normal_j = 0;
                   }
                   btVector3 normal_force = normal_j * normal;
                   
                   body1->setLinearVelocity(body1->getLinearVelocity() + body1->getInvMass() * normal_force);
                   body1->setAngularVelocity(body1->getAngularVelocity() + r1.cross(normal_force) * body1->getInvInertiaTensorWorld());
                   body2->setLinearVelocity(body2->getLinearVelocity() - body2->getInvMass() * normal_force);
                   body2->setAngularVelocity(body2->getAngularVelocity() - r2.cross(normal_force) * body2->getInvInertiaTensorWorld());
                   
                   // tangent force (friction)
                   btVector3 tangent = (v1-v2) - (vrel * normal);
                   
                   if(!btFuzzyZero(tangent.x()) || !btFuzzyZero(tangent.y()) || !btFuzzyZero(tangent.z())) {
                       tangent.normalize();
                   }
                   
                   btScalar tangent_numerator = -(tangent.dot(v1-v2));
                   btScalar tangent_term1 = body1->getInvMass();
                   btScalar tangent_term2 = body2->getInvMass();
                   btScalar tangent_term3 = tangent.dot((body1->getInvInertiaTensorWorld() * r1.cross(tangent)).cross(r1));
                   btScalar tangent_term4 = tangent.dot((body2->getInvInertiaTensorWorld() * r2.cross(tangent)).cross(r2));
                   
                   btScalar tangent_j = tangent_numerator / (tangent_term1 + tangent_term2 + tangent_term3 + tangent_term4);
                   
                   btScalar maxTangentForce = rigid_friction * normal_j;
                   if(tangent_j > maxTangentForce) {
                       tangent_j = maxTangentForce;
                   } else if (tangent_j < -maxTangentForce) {
                       tangent_j = -maxTangentForce;
                   }
                   btVector3 tangent_force = tangent_j * tangent;
                   
                   body1->setLinearVelocity(body1->getLinearVelocity() + body1->getInvMass() * tangent_force);
                   body1->setAngularVelocity(body1->getAngularVelocity() + r1.cross(tangent_force) * body1->getInvInertiaTensorWorld());
                   body2->setLinearVelocity(body2->getLinearVelocity() - body2->getInvMass() * tangent_force);
                   body2->setAngularVelocity(body2->getAngularVelocity() - r2.cross(tangent_force) * body2->getInvInertiaTensorWorld());
               }
           }
       }
    }
}

void Simulation::drawGround() {
	float speccolor[4] = {1.0f, 1.0f, 1.0f, 1.0f};
	btBoxShape *box = (btBoxShape *)ground->getCollisionShape();
	btVector3 position = ground->getCenterOfMassPosition();
	btVector3 fullExtents = box->getHalfExtentsWithoutMargin() * 2;
	
	glPushMatrix();
		float color[4] = {0.2f, 0.2f, 0.2f, 1.0f};
		glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
		glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
		glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
		glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
		glTranslatef(position.x(), position.y(), position.z());
		glScalef(fullExtents.x(), fullExtents.y(), fullExtents.z());
		glutSolidCube(1.0f);
	glPopMatrix();
}
void Simulation::drawFinger1() {
	float speccolor[4] = {1.0f, 1.0f, 1.0f, 1.0f};
	btBoxShape *shape = (btBoxShape *)finger1->getCollisionShape();
	btVector3 position = finger1->getCenterOfMassPosition();
    btQuaternion orientation = finger1->getCenterOfMassTransform().getRotation();
    btScalar rot_angle = orientation.getAngle();
    btVector3 rot_axis = orientation.getAxis();
	btVector3 fullExtents = shape->getHalfExtentsWithoutMargin() * 2;

	glPushMatrix();
		float color[4] = {1.0f, 0.0f, 0.0f, 1.0f};
		glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
		glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
		glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
		glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
		glTranslatef(position.x(), position.y(), position.z());
        glScalef(fullExtents.x(), fullExtents.y(), fullExtents.z());
        glRotatef(rot_angle*(180/M_PI), rot_axis.x(), rot_axis.y(), rot_axis.z());
		glutSolidCube(1.0f);
	glPopMatrix();
}
void Simulation::drawFinger2() {
	float speccolor[4] = {1.0f, 1.0f, 1.0f, 1.0f};
	btBoxShape *shape = (btBoxShape *)finger2->getCollisionShape();
	btVector3 position = finger2->getCenterOfMassPosition();
    btQuaternion orientation = finger2->getCenterOfMassTransform().getRotation();
    btScalar rot_angle = orientation.getAngle();
    btVector3 rot_axis = orientation.getAxis();
	btVector3 fullExtents = shape->getHalfExtentsWithoutMargin() * 2;
    
	glPushMatrix();
        float color[4] = {1.0f, 0.0f, 0.0f, 1.0f};
        glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
        glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
        glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
        glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
        glTranslatef(position.x(), position.y(), position.z());
        glScalef(fullExtents.x(), fullExtents.y(), fullExtents.z());
        glRotatef(rot_angle*(180/M_PI), rot_axis.x(), rot_axis.y(), rot_axis.z());
        glutSolidCube(1.0f);
	glPopMatrix();
}
void Simulation::drawBlock() {
	float speccolor[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    btBoxShape *shape = (btBoxShape *)block->getCollisionShape();
	btVector3 position = block->getCenterOfMassPosition();
    btQuaternion orientation = block->getCenterOfMassTransform().getRotation();
    btScalar rot_angle = orientation.getAngle();
    btVector3 rot_axis = orientation.getAxis();
	btVector3 fullExtents = shape->getHalfExtentsWithoutMargin() * 2;
    
	glPushMatrix();
        float color[4] = {0.0f, 0.0f, 1.0f, 1.0f};
        glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
        glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
        glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
        glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
        
        glTranslatef(position.x(), position.y(), position.z());
        glScalef(fullExtents.x(), fullExtents.y(), fullExtents.z());
        glRotatef(rot_angle*(180/M_PI), rot_axis.x(), rot_axis.y(), rot_axis.z());
        glutSolidCube(1.0f);
	glPopMatrix();
}
void Simulation::drawSoftCube() {
    float speccolor[4] = {1.0f, 1.0f, 1.0f, 1.0f};
	btBoxShape *shape = (btBoxShape *)softCube->m_rigidBody->getCollisionShape();
	btVector3 position = softCube->m_rigidBody->getCenterOfMassPosition();
    btQuaternion orientation = softCube->m_rigidBody->getCenterOfMassTransform().getRotation();
    btScalar rot_angle = orientation.getAngle();
    btVector3 rot_axis = orientation.getAxis();
	btVector3 fullExtents = shape->getHalfExtentsWithoutMargin() * 2;
    
	glPushMatrix();
        float color[4] = {1.0f, 0.0f, 0.0f, 1.0f};
        glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
        glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
        glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
        glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
        glTranslatef(position.x(), position.y(), position.z());
        glScalef(fullExtents.x(), fullExtents.y(), fullExtents.z());
        glRotatef(rot_angle*(180/M_PI), rot_axis.x(), rot_axis.y(), rot_axis.z());
        glutSolidCube(1.0f);
	glPopMatrix();
    
    vector<ParticleGroup*>::iterator g;
    for(g = softCube->m_particleGroups.begin(); g != softCube->m_particleGroups.end(); g++)
    {
        drawParticleGroup(**g);
    }
}
void Simulation::drawParticleGroup(ParticleGroup &group) {
    
    drawParticle(*group.m_particles.at(0));
    
    btVector3 tmpCenter = group.m_particles.at(0)->position;
    
    float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
    float color[4] = {0.0f, 0.0f, 1.0f, 1.0f};
    
    for(int i = 1; i < group.m_particles.size(); i++) {
        
        Particle *p = group.m_particles.at(i);
        
        btVector3 tmpPos = p->position;
        
        drawParticle(*p);
        
        glPushMatrix();
            glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
            glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
            glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
            glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
            glColor3fv(color);

            glLineWidth (3.0f);

            glBegin (GL_LINES);
                glVertex3f(tmpPos.x(), tmpPos.y(), tmpPos.z());
                glVertex3f(tmpCenter.x(), tmpCenter.y(), tmpCenter.z());
            glEnd ();

            glLineWidth (1);
        glPopMatrix();
    }
}
void Simulation::drawParticle(Particle &p) {
    float* speccolor;
    float* color;
    if(p.hasCollided) {
        float tmp1[4] = {1.0f, 0.0f, 0.0f, 1.0f};
        speccolor = tmp1;
        float tmp2[4] = {1.0f, 0.0f, 0.0f, 1.0f};
        color = tmp2;
    } else {
        float tmp1[4] = {1.0f, 1.0f, 0.0f, 1.0f};
        speccolor = tmp1;
        float tmp2[4] = {1.0f, 1.0f, 0.0f, 1.0f};
        color = tmp2;
    }
	btVector3 position = p.position;
    btQuaternion orientation = p.orientation;
    btScalar rot_angle = orientation.getAngle();
    btVector3 rot_axis = orientation.getAxis();
	btVector3 fullExtents = p.radii;
    
	glPushMatrix();
        glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
        glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
        glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
        glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
        glTranslatef(position.x(), position.y(), position.z());
        glRotatef(rot_angle*(180/M_PI), rot_axis.x(), rot_axis.y(), rot_axis.z());
        glScalef(fullExtents.x(), fullExtents.y(), fullExtents.z());
        glutSolidSphere(1.0f, 20.0f, 20.0f);
	glPopMatrix();
}


void Simulation::render() {
	
	drawGround();
    drawBlock();
	
	if (useDeformable) {
        drawSoftCube();
	} else {
		drawFinger1();
		drawFinger2();
	    drawBlock();
    }
}

void Simulation::predictUnconstrainedMotion(float timeStep) {
    
    block->applyGravity();
    block->integrateVelocities(timeStep);
    block->predictIntegratedTransform(timeStep, block->getInterpolationWorldTransform());
    
    if(useDeformable) {
        
        softCube->applyGravity(timeStep);
        softCube->integrateVelocities(timeStep);
        softCube->predictIntegratedTransform(timeStep, softCube->m_rigidBody->getInterpolationWorldTransform());

    } else {
        block->applyGravity();
        block->integrateVelocities(timeStep);
        block->predictIntegratedTransform(timeStep, block->getInterpolationWorldTransform());
        
        finger1->applyCentralForce(btVector3(20.0f, 10.0f, 0.0f));
        finger1->integrateVelocities(timeStep);
        finger1->predictIntegratedTransform(timeStep, finger1->getInterpolationWorldTransform());
        
        finger2->applyCentralForce(btVector3(-20.0f, 10.0f, 0.0f));
        finger2->integrateVelocities(timeStep);
        finger2->predictIntegratedTransform(timeStep, finger2->getInterpolationWorldTransform());
    }
}

void Simulation::integrateTransforms(float timeStep) {
    
    btTransform predictedTrans;
    block->predictIntegratedTransform(timeStep, predictedTrans);
    block->proceedToTransform(predictedTrans);
    
    if(useDeformable) {
        
        softCube->integrateTransforms(timeStep);
        
    } else {
        
        btVector3 fv1 = finger1->getLinearVelocity();
        finger1->setLinearVelocity(btVector3(fv1.x(),fv1.y(),0));
        finger1->setAngularVelocity(btVector3(0,0,0));
        finger1->predictIntegratedTransform(timeStep, predictedTrans);
        finger1->proceedToTransform(predictedTrans);
        
        btVector3 fv2 = finger2->getLinearVelocity();
        finger2->setLinearVelocity(btVector3(fv2.x(),fv2.y(),0));
        finger2->setAngularVelocity(btVector3(0,0,0));
        finger2->predictIntegratedTransform(timeStep, predictedTrans);
        finger2->proceedToTransform(predictedTrans);
    }
}

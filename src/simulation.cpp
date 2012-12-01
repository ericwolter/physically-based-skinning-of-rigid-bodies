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
    btCollisionShape *blockShape = new btBoxShape(btVector3(1.0f, 6.0f, 6.0f));
    
    btTransform blockTransform;
    blockTransform.setIdentity();
    blockTransform.setOrigin(btVector3(0.0f,6.5f,0.0f));
    
    btScalar blockMass(0.01f);
    btVector3 blockLocalInertia(0,0,0);
    
    blockShape->calculateLocalInertia(blockMass, blockLocalInertia);
    btDefaultMotionState *blockMotionState = new btDefaultMotionState(blockTransform);
    btRigidBody::btRigidBodyConstructionInfo blockInfo(blockMass, blockMotionState, blockShape, blockLocalInertia);
    
    block = new btRigidBody(blockInfo);
    block->setGravity(gravity);
    
    if (useDeformable) {
        // setup soft finger 1
        btCollisionShape *softFinger1Shape = new btBoxShape(btVector3(1.0f, 1.0f, 1.0f));
        
        btTransform softFinger1Transform;
        softFinger1Transform.setIdentity();
        softFinger1Transform.setOrigin(btVector3(-3.5,3,0));
        softFinger1Transform.setRotation(btQuaternion(btVector3(0,1,0), 0.1));
        
        btScalar softFinger1Mass(1.0f);
        btVector3 softFinger1LocalInertia(0,0,0);
        
        softFinger1Shape->calculateLocalInertia(softFinger1Mass, softFinger1LocalInertia);
        btDefaultMotionState* softFinger1MotionState = new btDefaultMotionState(softFinger1Transform);
        btRigidBody::btRigidBodyConstructionInfo softFinger1Info(softFinger1Mass, softFinger1MotionState, softFinger1Shape, softFinger1LocalInertia);
        
        btRigidBody *rigidFinger1 = new btRigidBody(softFinger1Info);
        rigidFinger1->setGravity(gravity);
        
        softFinger1 = new CombinedBody(rigidFinger1);
        
        // setup soft finger 2
        btCollisionShape *softFinger2Shape = new btBoxShape(btVector3(1.0f, 1.0f, 1.0f));
        
        btTransform softFinger2Transform;
        softFinger2Transform.setIdentity();
        softFinger2Transform.setOrigin(btVector3(3.5,3,0));
        softFinger2Transform.setRotation(btQuaternion(btVector3(0,1,0), -0.1));
        
        btScalar softFinger2Mass(1.0f);
        btVector3 softFinger2LocalInertia(0,0,0);
        
        softFinger2Shape->calculateLocalInertia(softFinger2Mass, softFinger2LocalInertia);
        btDefaultMotionState* softFinger2MotionState = new btDefaultMotionState(softFinger2Transform);
        btRigidBody::btRigidBodyConstructionInfo softFinger2Info(softFinger2Mass, softFinger2MotionState, softFinger2Shape, softFinger2LocalInertia);
        
        btRigidBody *rigidFinger2 = new btRigidBody(softFinger2Info);
        rigidFinger2->setGravity(gravity);
        
        softFinger2 = new CombinedBody(rigidFinger2);
        
    } else {
        // setup finger 1
        btCollisionShape *finger1Shape = new btBoxShape(btVector3(1.0f, 1.0f, 3.0f));
        
        btTransform finger1Transform;
        finger1Transform.setIdentity();
        finger1Transform.setOrigin(btVector3(-3.5f,3,0.0f));
        finger1Transform.setRotation(btQuaternion(btVector3(0,1,0), 0.1));
        
        btScalar finger1Mass(1.52f);
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
        finger2Transform.setOrigin(btVector3(3.5f,3,0.0f));
        finger2Transform.setRotation(btQuaternion(btVector3(0,1,0), -0.1));
        
        btScalar finger2Mass(1.52f);
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
        int numSoftBodies = 2;
        CombinedBody* bodies[2] = {softFinger1, softFinger2};
        
        for(int i=0; i < numSoftBodies; i++) {
            for(int j = 0; j < bodies[i]->m_outerParticles.size(); j++) {
                bodies[i]->m_outerParticles.at(j)->hasCollided = false;
            }
            
            //softCollisionWithGround(timeStep, bodies[i]);
            softCollisionWithBlock(timeStep, bodies[i]);
        }
        
        // gauss seidel
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < numSoftBodies; j++) {
                CombinedBody *body = bodies[j];
                // shape matching
                vector<ParticleGroup*>::iterator g;
                for(g = body->m_particleGroups.begin(); g != body->m_particleGroups.end(); g++) {
                    (*g)->update();
                }
            }
        }
        
        for(int i = 0; i < numSoftBodies; i++) {
            realignAttachedParticles(timeStep, bodies[i]);
        }
    }
	collision(timeStep);
	
	integrateTransforms(timeStep);
    
    ground->clearForces();
    block->clearForces();
    
    if(useDeformable) {
        softFinger1->m_rigidBody->clearForces();
        softFinger2->m_rigidBody->clearForces();
    } else {
        finger1->clearForces();
        finger2->clearForces();
        block->clearForces();
    }
}

void Simulation::realignAttachedParticles(float timeStep, CombinedBody *body) {
    
    for(int i = 0; i < body->m_attachedParticles.size(); i++) {
        Particle *particle = body->m_attachedParticles.at(i);
        
        btTransform bodyTransform = body->m_rigidBody->getWorldTransform();
        btVector3 truePosition = bodyTransform * particle->relativePosition;
        
        btVector3 particlePosition = particle->predictedPosition;
        btVector3 delta = particlePosition - truePosition;
        btVector3 impulse = (particle->mass * delta) / timeStep;
        
        btVector3 bodyVelocity = body->m_rigidBody->getLinearVelocity();
        body->m_rigidBody->applyImpulse(impulse, particle->relativePosition);
        bodyVelocity = body->m_rigidBody->getLinearVelocity();
        particle->predictedPosition = truePosition;
    }
}

// see: http://stackoverflow.com/questions/4578967/cube-sphere-intersection-test
bool Simulation::quickCollisionCheck(btRigidBody *body, Particle *particle) {
    
    // axis aligned bounding box of the body
    btVector3 a, b;
    body->getAabb(a, b);
    
    // center c and radius r of particle sphere
    btVector3 c = particle->predictedPosition;
    btScalar r = particle->radii[particle->radii.maxAxis()];
    
    float dist_squared = pow(r,2);
    if (c.x() < a.x()) {
        dist_squared -= pow(c.x() - a.x(), 2);
    } else if (c.x() > b.x()) {
        dist_squared -= pow(c.x() - b.x(), 2);
    }
    if (c.y() < a.y()) {
        dist_squared -= pow(c.y() - a.y(), 2);
    } else if (c.y() > b.y()) {
        dist_squared -= pow(c.y() - b.y(), 2);
    }
    if (c.z() < a.z()) {
        dist_squared -= pow(c.z() - a.z(), 2);
    } else if (c.z() > b.z()) {
        dist_squared -= pow(c.z() - b.z(), 2);
    }
    
    return dist_squared > 0;
}

void Simulation::softCollisionWithGround(float timeStep, CombinedBody *body) {
    
    for(int i=0; i< body->m_outerParticles.size(); i++)
    {
        Particle *particle = body->m_outerParticles.at(i);
        
        btMatrix3x3 R = btMatrix3x3(particle->predictedOrientation);
        btMatrix3x3 AInv = R * particle->radiiMatrix * R.transpose();
        
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
        }
    }
}

void Simulation::softCollisionWithBlock(float timeStep, CombinedBody *body) {
    
    for(int i=0; i< body->m_outerParticles.size(); i++)
    {
        Particle *particle = body->m_outerParticles.at(i);
        
        btMatrix3x3 R = btMatrix3x3(particle->predictedOrientation);
        btMatrix3x3 AInv = R * particle->radiiMatrix * R.transpose();
        
        // collision with block
        btBoxShape *blockShape = (btBoxShape *)block->getCollisionShape();
        int numPlanes = blockShape->getNumPlanes();
        for(int pI = 0; pI < numPlanes; pI++) {
            btVector4 plane;
            blockShape->getPlaneEquation(plane, pI);

            btVector3 n = btVector3(plane.x(), plane.y(), plane.z());
            btScalar d = -plane.w();
            btVector3 p = d * n;

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
                if(blockShape->isInside(bodyX1, margin)) {
                    particle->hasCollided = true;
                    btVector3 delta = n * (d - d1);
                    particle->predictedPosition += delta;

                    CollisionResponse res = collisionResolution(timeStep, x1, n, (d-d1), particle->predictedPosition, t.getOrigin(), particle->linearVelocity, block->getLinearVelocity(), particle->angularVelocity, block->getAngularVelocity(), 1/particle->mass, block->getInvMass(), particle->getInvInertiaTensorWorld(), block->getInvInertiaTensorWorld());
                    
                    particle->linearVelocity += res.normal1_linear;
                    particle->angularVelocity += res.normal1_angular;
                    block->setLinearVelocity(block->getLinearVelocity() - res.normal2_linear);
                    block->setAngularVelocity(block->getAngularVelocity() - res.normal2_angular);
                    
                    particle->linearVelocity += res.friction1_linear;
                    particle->angularVelocity += res.friction1_angular;
                    block->setLinearVelocity(block->getLinearVelocity() - res.friction2_linear);
                    block->setAngularVelocity(block->getAngularVelocity() - res.friction2_angular);
                    block->applyImpulse((particle->mass * -delta) / timeStep, t.getOrigin() - x1);
                }
            } else {
                if (blockShape->isInside(bodyX2, margin)) {
                    particle->hasCollided = true;
                    btVector3 delta = n * (d - d2);
                    particle->predictedPosition += delta;
                    
                    CollisionResponse res = collisionResolution(timeStep, x2, n, (d-d2), particle->predictedPosition, t.getOrigin(), particle->linearVelocity, block->getLinearVelocity(), particle->angularVelocity, block->getAngularVelocity(), 1/particle->mass, block->getInvMass(), particle->getInvInertiaTensorWorld(), block->getInvInertiaTensorWorld());
                    
                    particle->linearVelocity += res.normal1_linear;
                    particle->angularVelocity += res.normal1_angular;
                    block->setLinearVelocity(block->getLinearVelocity() - res.normal2_linear);
                    block->setAngularVelocity(block->getAngularVelocity() - res.normal2_angular);
                    
                    particle->linearVelocity += res.friction1_linear;
                    particle->angularVelocity += res.friction1_angular;
                    block->setLinearVelocity(block->getLinearVelocity() - res.friction2_linear);
                    block->setAngularVelocity(block->getAngularVelocity() - res.friction2_angular);
                }
            }
        }
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
           
    for(int s = 0; s < 1; s++)
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
                   
                   CollisionResponse res = collisionResolution(timeStep, hitPoint, normal, penDepth, body1->getCenterOfMassPosition(), body2->getCenterOfMassPosition(), body1->getLinearVelocity(), body2->getLinearVelocity(), body1->getAngularVelocity(), body2->getAngularVelocity(), body1->getInvMass(), body2->getInvMass(), body1->getInvInertiaTensorWorld(), body2->getInvInertiaTensorWorld());
                   
                   // normal
                   body1->setLinearVelocity(body1->getLinearVelocity() + res.normal1_linear);
                   body1->setAngularVelocity(body1->getAngularVelocity() + res.normal1_angular);
                   body2->setLinearVelocity(body2->getLinearVelocity() - res.normal2_linear);
                   body2->setAngularVelocity(body2->getAngularVelocity() - res.normal2_angular);
                   
                   // friction
                   body1->setLinearVelocity(body1->getLinearVelocity() + res.friction1_linear);
                   body1->setAngularVelocity(body1->getAngularVelocity() + res.friction1_angular);
                   body2->setLinearVelocity(body2->getLinearVelocity() - res.friction2_linear);
                   body2->setAngularVelocity(body2->getAngularVelocity() - res.friction2_angular);
               }
           }
       }
    }
}

Simulation::CollisionResponse Simulation::collisionResolution(float timeStep, btVector3 contactPoint, btVector3 contactNormal, btScalar contactDepth, btVector3 p1, btVector3 p2, btVector3 v1, btVector3 v2, btVector3 a1, btVector3 a2, btScalar invM1, btScalar invM2, btMatrix3x3 invT1, btMatrix3x3 invT2) {
    
    CollisionResponse res;
    
    btVector3 r1 = contactPoint - p1;
    btVector3 r2 = contactPoint - p2;
    
    btVector3 rv1 = v1 + (a1.cross(r1));
    btVector3 rv2 = v2 + (a2.cross(r2));
    
    btScalar vrel = contactNormal.dot(rv1 - rv2);
    
    // src: http://www.xbdev.net/physics/RigidBodyImpulseCubes/index.php
    // normal force
    btScalar allowedPenetration = 0.1f;
    btScalar biasFactor = 0.3f;
    btScalar bias;
    if ((contactDepth - allowedPenetration) < 0.0f) {
        bias = 0;
    }
    else
    {
        bias = biasFactor * (1/timeStep) * (contactDepth - allowedPenetration);
    }
    
    btScalar normal_numerator = (-vrel + bias);
    btScalar normal_term1 = invM1;
    btScalar normal_term2 = invM2;
    btScalar normal_term3 = contactNormal.dot((invT1 * r1.cross(contactNormal)).cross(r1));
    btScalar normal_term4 = contactNormal.dot((invT2 * r2.cross(contactNormal)).cross(r2));
    
    btScalar normal_j = normal_numerator / (normal_term1 + normal_term2 + normal_term3 + normal_term4);
    if (normal_j < 0)
    {
        normal_j = 0;
    }
    
    btVector3 normal_force = normal_j * contactNormal;
    
    res.normal1_linear = invM1 * normal_force;
    res.normal1_angular = r1.cross(normal_force) * invT1;
    res.normal2_linear = invM2 * normal_force;
    res.normal2_angular = r2.cross(normal_force) * invT2;
    
    // tangent force (friction)
    btVector3 tangent = (rv1-rv2) - (vrel * contactNormal);
    
    if(!btFuzzyZero(tangent.x()) || !btFuzzyZero(tangent.y()) || !btFuzzyZero(tangent.z())) {
        tangent.normalize();
    }
    
    btScalar tangent_numerator = -(tangent.dot(rv1-rv2));
    btScalar tangent_term1 = invM1;
    btScalar tangent_term2 = invM2;
    btScalar tangent_term3 = tangent.dot((invT1 * r1.cross(tangent)).cross(r1));
    btScalar tangent_term4 = tangent.dot((invT2 * r2.cross(tangent)).cross(r2));
    
    btScalar tangent_j = tangent_numerator / (tangent_term1 + tangent_term2 + tangent_term3 + tangent_term4);
    
    btScalar maxTangentForce = rigid_friction * normal_j;
    if(tangent_j > maxTangentForce) {
        tangent_j = maxTangentForce;
    } else if (tangent_j < -maxTangentForce) {
        tangent_j = -maxTangentForce;
    }
    
    btVector3 tangent_force = tangent_j * tangent;
    
    res.friction1_linear = invM1 * tangent_force;
    res.friction1_angular = r1.cross(tangent_force) * invT1;
    res.friction2_linear = invM2 * tangent_force;
    res.friction2_angular = r2.cross(tangent_force) * invT2;
    
    return res;
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
void Simulation::drawSoftFinger(CombinedBody *finger) {
    float speccolor[4] = {1.0f, 1.0f, 1.0f, 1.0f};
	btBoxShape *shape = (btBoxShape *)finger->m_rigidBody->getCollisionShape();
	btVector3 position = finger->m_rigidBody->getCenterOfMassPosition();
    btQuaternion orientation = finger->m_rigidBody->getCenterOfMassTransform().getRotation();
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
    for(g = finger->m_particleGroups.begin(); g != finger->m_particleGroups.end(); g++)
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
        drawSoftFinger(softFinger1);
        drawSoftFinger(softFinger2);
	} else {
		drawFinger1();
		drawFinger2();
    }
}

void Simulation::predictUnconstrainedMotion(float timeStep) {
    
    block->applyGravity();
    block->integrateVelocities(timeStep);
    block->predictIntegratedTransform(timeStep, block->getInterpolationWorldTransform());
    
    if(useDeformable) {
        
//        softFinger1->applyGravity(timeStep);
        softFinger1->m_rigidBody->applyCentralForce(btVector3(20.0f, 10.0f, 0.0f));
        softFinger1->integrateVelocities(timeStep);
        softFinger1->predictIntegratedTransform(timeStep, softFinger1->m_rigidBody->getInterpolationWorldTransform());
//        softFinger2->applyGravity(timeStep);
        softFinger2->m_rigidBody->applyCentralForce(btVector3(-20.0f, 10.0f, 0.0f));
        softFinger2->integrateVelocities(timeStep);
        softFinger2->predictIntegratedTransform(timeStep, softFinger2->m_rigidBody->getInterpolationWorldTransform());

    } else {
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
        btVector3 fv1 = softFinger1->m_rigidBody->getLinearVelocity();
        softFinger1->m_rigidBody->setLinearVelocity(btVector3(fv1.x() < 0 ? 0 : fv1.x(), fv1.y() < 0 ? 0: fv1.y(),0));
        softFinger1->m_rigidBody->setAngularVelocity(btVector3(0, 0,0));
        softFinger1->integrateTransforms(timeStep);
        btVector3 fv2 = softFinger2->m_rigidBody->getLinearVelocity();
        softFinger2->m_rigidBody->setLinearVelocity(btVector3(fv2.x() > 0 ? 0 : fv2.x(), fv1.y() < 0 ? 0: fv1.y(),0));
        softFinger2->m_rigidBody->setAngularVelocity(btVector3(0, 0,0));
        softFinger2->integrateTransforms(timeStep);
    } else {
        btVector3 fv1 = finger1->getLinearVelocity();
        finger1->setLinearVelocity(btVector3(fv1.x() < 0 ? 0 : fv1.x(),fv1.y(),0));
        finger1->setAngularVelocity(btVector3(0,0,0));
        finger1->predictIntegratedTransform(timeStep, predictedTrans);
        finger1->proceedToTransform(predictedTrans);
        
        btVector3 fv2 = finger2->getLinearVelocity();
        finger2->setLinearVelocity(btVector3(fv2.x() > 0 ? 0 : fv2.x(),fv1.y(),0));
        finger2->setAngularVelocity(btVector3(0,0,0));
        finger2->predictIntegratedTransform(timeStep, predictedTrans);
        finger2->proceedToTransform(predictedTrans);
    }
}

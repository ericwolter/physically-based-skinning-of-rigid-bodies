#ifndef __Thesis__simulation__
#define __Thesis__simulation__

#include <BulletDynamics/btBulletDynamicsCommon.h>

#include "combinedbody.h"

class Simulation
{
public:
	Simulation();
	~Simulation();
    
    struct CollisionResponse {
        btVector3 normal1_linear;
        btVector3 normal1_angular;
        btVector3 friction1_linear;
        btVector3 friction1_angular;
        btVector3 normal2_linear;
        btVector3 normal2_angular;
        btVector3 friction2_linear;
        btVector3 friction2_angular;
    };
	
	void init(bool useDeformable);
	void step(float timeStep);
	void render();
	
private:
	void predictUnconstrainedMotion(float timeStep);
	void collision(float timeStep);
    
    CollisionResponse collisionResolution(float timeStep, btVector3 contactPoint, btVector3 contactNormal, btScalar contactDepth, btVector3 p1, btVector3 p2, btVector3 v1, btVector3 v2, btVector3 a1, btVector3 a2, btScalar invM1, btScalar invM2, btMatrix3x3 invT1, btMatrix3x3 invT2);
    
    void softCollisionWithGround(float timeStep, CombinedBody *body);
    void softCollisionWithBlock(float timeStep, CombinedBody *body);
	void integrateTransforms(float timeStep);
    
    void realignAttachedParticles(float timeStep, CombinedBody *body);
	
	void drawGround();
	void drawFinger1();
	void drawFinger2();
	void drawBlock();
	
    void drawSoftFinger(CombinedBody *finger);

	void drawParticleGroup(ParticleGroup &g);
	void drawParticle(Particle &p);
    
	btDefaultCollisionConfiguration *collisionConfiguration;
    btCollisionDispatcher* dispatcher;
    btDispatcherInfo dispatchInfo;
	
	btRigidBody *ground;
	btRigidBody *finger1;
	btRigidBody *finger2;
	btRigidBody *block;
	
    CombinedBody *softFinger1;
    CombinedBody *softFinger2;
	
	bool useDeformable;
};

#endif /* defined(__Thesis__simulation__) */
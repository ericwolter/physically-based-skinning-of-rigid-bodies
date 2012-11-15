#ifndef __Thesis__simulation__
#define __Thesis__simulation__

#include <BulletDynamics/btBulletDynamicsCommon.h>

#include "combinedbody.h"

class Simulation
{
public:
	Simulation();
	~Simulation();
	
	void init(bool useDeformable);
	void step(float timeStep);
	void render();
	
private:
	void predictUnconstrainedMotion(float timeStep);
	void collision(float timeStep);
	void integrateTransforms(float timeStep);
	
	void drawGround();
	void drawFinger1();
	void drawFinger2();
	void drawBlock();
	
	void drawSoftCube();

	void drawParticleGroup(ParticleGroup &g);
	void drawParticle(Particle &p);
	
	btDefaultCollisionConfiguration *collisionConfiguration;
    btCollisionDispatcher* dispatcher;
    btDispatcherInfo dispatchInfo;
	
	btRigidBody *ground;
	btRigidBody *finger1;
	btRigidBody *finger2;
	btRigidBody *block;
	
	CombinedBody *softCube;
	
	bool useDeformable;
};

#endif /* defined(__Thesis__simulation__) */
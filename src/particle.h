#ifndef __Thesis__particle__
#define __Thesis__particle__

#include <BulletDynamics/btBulletDynamicsCommon.h>

class Particle
{
public:
	Particle();
	~Particle();
	
	bool hasCollided;
    
	float mass;
	btVector3 radii;
    btMatrix3x3 radiiMatrix;
	btVector3 restPosition;
	btVector3 relativePosition;
	btQuaternion restOrientation;
	
	btVector3 position;
	btQuaternion orientation;
	btVector3 linearVelocity;
	btVector3 angularVelocity;
	
	btVector3 predictedPosition;
	btQuaternion predictedOrientation;
	
	btMatrix3x3 getMomentMatrix();
    
    btMatrix3x3 getInvInertiaTensorWorld();
	
	void init();
    btTransform getTransform();
    btTransform getPredictedTransform();
    
	static float getSqrDistance(Particle *p1, Particle *p2);
};

#endif
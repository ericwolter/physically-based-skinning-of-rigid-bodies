#ifndef __Thesis__combinedbody__
#define __Thesis__combinedbody__

#include <BulletDynamics/btBulletDynamicsCommon.h>

#include "particlegroup.h"
#include "particle.h"

class CombinedBody
{
public:
    CombinedBody(btRigidBody *rigidBody);
    ~CombinedBody();
    
    btVector3 restPosition;
    
    vector<ParticleGroup*> m_particleGroups;
    vector<Particle*> m_particles;
    vector<Particle*> m_attachedParticles;
    vector<Particle*> m_outerParticles;
    
    btRigidBody *m_rigidBody;
    void applyGravity(float timeStep);
    void integrateVelocities(float timeStep);
    void predictIntegratedTransform(float timeStep, btTransform &predictedTransform);
    void integrateTransforms(float timeStep);
    
private:
    void addAttachedParticle(Particle *p);
    void addOuterParticle(Particle *p);
    
    bool vectorIsEqualWithinMargin(btVector3 v1, btVector3 v2, float margin);
};

#endif
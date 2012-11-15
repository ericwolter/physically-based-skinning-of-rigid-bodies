#include "combinedbody.h"

const int density = 1;
const int groupSize = 4;
const float extrude = 2.0f;

CombinedBody::CombinedBody(btRigidBody *rigidBody)
{
    m_rigidBody = rigidBody;
    btVector3 origin = this->m_rigidBody->getWorldTransform().getOrigin();
    restPosition = origin;
    
    if(m_rigidBody->isActive() && (!m_rigidBody->isStaticObject()))
    {
        if(m_rigidBody->getCollisionShape()->getShapeType() == BOX_SHAPE_PROXYTYPE)
        {
            btBoxShape *box = (btBoxShape *)m_rigidBody->getCollisionShape();
            
            btVector3 halfExtents = box->getHalfExtentsWithoutMargin();
            btVector3 fullExtent = halfExtents * 2;
            
            btVector3 offset = fullExtent / (density + 1);
            
//            Particle *p1 = new Particle();
//            p1->mass = 0.01f;
//            p1->radii = btVector3(0.3f, 0.1f, 0.3f);
//            p1->relativePosition = btVector3(0, 0, 0);
//            p1->position = origin + p1->relativePosition;
//            p1->orientation = btQuaternion(btVector3(0,0,1), 90*(M_PI/180));
//            addOuterParticle(p1);
//            Particle *p2 = new Particle();
//            p2->mass = 0.01f;
//            p2->radii = btVector3(0.3f, 0.1f, 0.3f);
//            p2->relativePosition = btVector3(1, 0, 0);
//            p2->position = origin + p2->relativePosition;
//            p2->orientation = btQuaternion(btVector3(0,0,1), 90*(M_PI/180));
//            addOuterParticle(p2);
//            
//            ParticleGroup *g = new ParticleGroup();
//            g->m_particles.push_back(p1);
//            g->m_particles.push_back(p2);
//            m_particleGroups.push_back(g);
            
            // TopSide
            for(float x = 0; x <= fullExtent.x(); x+=offset.x())
            {
                for (float z = 0; z <= fullExtent.z(); z+=offset.z())
                {
                    Particle *p = new Particle();
                    p->mass = 0.01f;
                    p->radii = btVector3(0.3f, 0.1f, 0.3f);
                    p->relativePosition = btVector3(-halfExtents.x()+x, halfExtents.y(), -halfExtents.z()+z);
                    p->position = origin + p->relativePosition;
                    p->orientation = btQuaternion(btVector3(1,0,0), 0*(M_PI/180));
                    addAttachedParticle(p);
                }
            }
            // LeftSide
            for(float y = 0; y <= fullExtent.y(); y+=offset.y())
            {
                for (float z = 0; z <= fullExtent.z(); z+=offset.z())
                {
                    Particle *p = new Particle();
                    p->mass = 0.01f;
                    p->radii = btVector3(0.3f, 0.1f, 0.3f);
                    p->relativePosition = btVector3(-halfExtents.x(), -halfExtents.y()+y, -halfExtents.z()+z);
                    p->position = origin + p->relativePosition;
                    p->orientation = btQuaternion(btVector3(0,0,1), 0*(M_PI/180));
                    addAttachedParticle(p);
                }
            }
            // BackSide
            for(float x = 0; x <= fullExtent.x(); x+=offset.x())
            {
                for (float y = 0; y <= fullExtent.y(); y+=offset.y())
                {
                    Particle *p = new Particle();
                    p->mass = 0.01f;
                    p->radii = btVector3(0.3f, 0.1f, 0.3f);
                    p->relativePosition = btVector3(-halfExtents.x()+x, -halfExtents.y()+y, halfExtents.z());
                    p->position = origin + p->relativePosition;
                    p->orientation = btQuaternion(btVector3(1,0,0), 0*(M_PI/180));
                    addAttachedParticle(p);
                }
            }
            // RightSide
            for(float y = 0; y <= fullExtent.y(); y+=offset.y())
            {
                for (float z = 0; z <= fullExtent.z(); z+=offset.z())
                {
                    Particle *p = new Particle();
                    p->mass = 0.01f;
                    p->radii = btVector3(0.3f, 0.1f, 0.3f);
                    p->relativePosition = btVector3(halfExtents.x(), -halfExtents.y()+y, -halfExtents.z()+z);
                    p->position = origin + p->relativePosition;
                    p->orientation = btQuaternion(btVector3(0,0,1), 0*(M_PI/180));
                    addAttachedParticle(p);
                }
            }
            // FrontSide
            for(float x = 0; x <= fullExtent.x(); x+=offset.x())
            {
                for (float y = 0; y <= fullExtent.y(); y+=offset.y())
                {
                    Particle *p = new Particle();
                    p->mass = 0.01f;
                    p->radii = btVector3(0.3f, 0.1f, 0.3f);
                    p->relativePosition = btVector3(-halfExtents.x()+x, -halfExtents.y()+y, -halfExtents.z());
                    p->position = origin + p->relativePosition;
                    p->orientation = btQuaternion(btVector3(1,0,0), 0*(M_PI/180));
                    addAttachedParticle(p);
                }
            }
            // BottomSide
            for(float x = 0; x <= fullExtent.x(); x+=offset.x())
            {
                for (float z = 0; z <= fullExtent.z(); z+=offset.z())
                {
                    Particle *p = new Particle();
                    p->mass = 0.01f;
                    p->radii = btVector3(0.3f, 0.1f, 0.3f);
                    p->relativePosition = btVector3(-halfExtents.x()+x, -halfExtents.y(), -halfExtents.z()+z);
                    p->position = origin + p->relativePosition;
                    p->orientation = btQuaternion(btVector3(1,0,0), 0*(M_PI/180));
                    addAttachedParticle(p);
                }
            }
            
            // create inner particle groups
            vector<Particle*>::iterator p;
            for(p = m_attachedParticles.begin(); p != m_attachedParticles.end(); p++) {
                Particle *oP = new Particle();
                oP->mass = 0.01f;
                oP->radii = btVector3(0.3f, 0.1f, 0.3f);
                btVector3 origin = this->m_rigidBody->getWorldTransform().getOrigin();
                oP->position = origin + ((*p)->position - origin) * extrude;
                oP->orientation = (*p)->orientation;
                addOuterParticle(oP);
            }
            
            for(p = m_attachedParticles.begin(); p!= m_attachedParticles.end(); p++)
            {
                Particle *currentCenterParticle = *p;
                
                // sort all particles by distance to current center
                vector<Particle*> sorted;
                vector<Particle*>::iterator p2;
                for(p2 = m_attachedParticles.begin(); p2 != m_attachedParticles.end(); p2++)
                {
                    btVector3 diff = currentCenterParticle->position - (*p2)->position;
                    float distanceToCenter = diff.length2();
                    
                    int position = 0;
                    for(int i = 1; i <= sorted.size(); i++)
                    {
                        position = i;
                        Particle *sortedParticle = sorted[i-1];
                        if (distanceToCenter < (currentCenterParticle->position - sortedParticle->position).length2())
                        {
                            position -= 1;
                            break;
                        }
                    }
                    
                    sorted.insert(sorted.begin() + position, *p2);
                }
                
                // create a group by taking n-nearest particles
                ParticleGroup *g = new ParticleGroup();
                for(int i = 0; i < groupSize; i++)
                {
                    g->m_particles.push_back(sorted.at(i));
                }
                
                // find corresponding outer particle
                for(p2 = m_outerParticles.begin(); p2 != m_outerParticles.end(); p2++)
                {
                    if (vectorIsEqualWithinMargin(origin + (currentCenterParticle->position - origin) * extrude, (*p2)->position, 0.0001f))
                    {
                        g->m_particles.push_back(*p2);
                        break;
                    }
                }
                
                m_particleGroups.push_back(g);
            }
            
            // create outer particle groups
            for(p = m_outerParticles.begin(); p!= m_outerParticles.end(); p++)
            {
                Particle *currentCenterParticle = *p;
                
                // sort all particles by distance to current center
                vector<Particle*> sorted;
                vector<Particle*>::iterator p2;
                for(p2 = m_outerParticles.begin(); p2 != m_outerParticles.end(); p2++)
                {
                    float distanceToCenter = (currentCenterParticle->position - (*p2)->position).length2();
                    
                    int position = 0;
                    for(int i = 1; i <= sorted.size(); i++)
                    {
                        position = i;
                        Particle *sortedParticle = sorted[i-1];
                        if (distanceToCenter < (currentCenterParticle->position - sortedParticle->position).length2())
                        {
                            position -= 1;
                            break;
                        }
                    }
                    
                    sorted.insert(sorted.begin() + position, *p2);
                }
                
                // create a group by taking n-nearest particles
                ParticleGroup *g = new ParticleGroup();
                for(int i = 0; i < groupSize; i++)
                {
                    g->m_particles.push_back(sorted.at(i));
                }
                
                // find corresponding inner particle
                for(p2 = m_attachedParticles.begin(); p2 != m_attachedParticles.end(); p2++)
                {
                    if (vectorIsEqualWithinMargin(currentCenterParticle->position, origin + ((*p2)->position - origin) * extrude, 0.1f))
                    {
                        g->m_particles.push_back(*p2);
                        break;
                    }
                }
                
                m_particleGroups.push_back(g);
            }
        }
    }
    
    vector<ParticleGroup*>::iterator pg;
    for(pg = m_particleGroups.begin(); pg != m_particleGroups.end(); pg++)
    {
        (*pg)->init();
    }
}

CombinedBody::~CombinedBody() {}

void CombinedBody::addAttachedParticle(Particle *newParticle)
{
    bool isNew = true;
    vector<Particle*>::iterator p;
    for(p=m_attachedParticles.begin(); p != m_attachedParticles.end(); p++)
    {
        if (!vectorIsEqualWithinMargin((*p)->position, newParticle->position, 0.0001f)) {
            isNew = true;
        }
        else
        {
            isNew = false;
            btQuaternion s = (*p)->orientation.slerp(newParticle->orientation, 0.5f);
            (*p)->orientation = s;
            break;
        }
    }
    
    if (isNew) {
        m_attachedParticles.push_back(newParticle);
        m_particles.push_back(newParticle);
    }
}

void CombinedBody::addOuterParticle(Particle *newParticle)
{
    bool isNew = true;
    vector<Particle*>::iterator p;
    for(p=m_outerParticles.begin(); p != m_outerParticles.end(); p++)
    {
        if (!vectorIsEqualWithinMargin((*p)->position, newParticle->position, 0.1f)) {
            isNew = true;
        }
        else
        {
            isNew = false;
            // combine orientations
            (*p)->orientation = (*p)->orientation * newParticle->orientation;
            break;
        }
    }
    
    if (isNew) {
        m_outerParticles.push_back(newParticle);
        m_particles.push_back(newParticle);
    }
}

bool CombinedBody::vectorIsEqualWithinMargin(btVector3 v1, btVector3 v2, float margin)
{
    float x1 = v1.x();
    float y1 = v1.y();
    float z1 = v1.z();
    float x2 = v2.x();
    float y2 = v2.y();
    float z2 = v2.z();
    
    float dx = fabs(x1 - x2);
    float dy = fabs(y1 - y2);
    float dz = fabs(z1 - z2);
    
    return dx < margin && dy < margin && dz < margin;
}

void CombinedBody::applyGravity(float timeStep)
{
    m_rigidBody->applyGravity();
    
    btVector3 gravity = m_rigidBody->getGravity();
    vector<Particle*>::iterator p;
    for(p = m_particles.begin(); p != m_particles.end(); p++)
    {
        (*p)->linearVelocity += timeStep * btVector3(gravity.x(), gravity.y(), gravity.z());
    }
}

void CombinedBody::integrateVelocities(float timeStep)
{
    m_rigidBody->integrateVelocities(timeStep);
}

void CombinedBody::predictIntegratedTransform(float timeStep, btTransform &predictedTransform)
{
    m_rigidBody->predictIntegratedTransform(timeStep, predictedTransform);
    
    vector<Particle*>::iterator p;
    for(p = m_particles.begin(); p != m_particles.end(); p++)
    {
        (*p)->predictedPosition = (*p)->position + (*p)->linearVelocity * timeStep;
        if ((*p)->angularVelocity.length() < 0.1f)
        {
            (*p)->predictedOrientation = (*p)->orientation;
        }
        else
        {
            btVector3 xyz = (*p)->angularVelocity.normalized() * sin(((*p)->angularVelocity.length()*timeStep)/2);
            btScalar w = cos(((*p)->angularVelocity.length()*timeStep)/2);
            btQuaternion tmp = btQuaternion(xyz.x(), xyz.y(), xyz.z(), w);
            (*p)->predictedOrientation = tmp * (*p)->orientation;
        }
    }
}

void CombinedBody::integrateTransforms(float timeStep)
{
    btTransform predictedTrans;
    if (m_rigidBody->isActive() && (!m_rigidBody->isStaticObject()))
    {
        m_rigidBody->predictIntegratedTransform(timeStep, predictedTrans);
        m_rigidBody->proceedToTransform(predictedTrans);
        
        vector<Particle*>::iterator p;
        for(p = m_particles.begin(); p != m_particles.end(); p++)
        {
            (*p)->linearVelocity = ((*p)->predictedPosition - (*p)->position) / timeStep;
            (*p)->position = (*p)->predictedPosition;
            
            // rotation transforming orientation to predicted orientation
            btQuaternion temp = (*p)->predictedOrientation * (*p)->orientation.inverse();
            // there are always two rotations, choose the shorter one
            if (temp.w() < 0)
            {
                temp = btQuaternion(-temp.x(), -temp.y(), -temp.z(), -temp.w());
            }
            
            btScalar angle = temp.getAngle();
            if (angle < 0.01f)
            {
                (*p)->angularVelocity = btVector3(0,0,0);
            }
            else
            {
                (*p)->angularVelocity = temp.getAxis() * (angle / timeStep);
            }
            
            (*p)->orientation = (*p)->predictedOrientation;
        }
    }
}

//
//  CombinedBody.cpp
//  BachelorThesis
//
//  Created by Eric Wolter on 9/28/12.
//  Copyright (c) 2012 Eric Wolter. All rights reserved.
//

#include <list>
#include "CombinedBody.h"

const int density = 1;
const int groupSize = 4;
const float extrude = 2.0f;

CombinedBody::CombinedBody(btRigidBody *rigidBody)
{
    m_rigidBody = rigidBody;
    btVector3 tmpOrigin = this->m_rigidBody->getWorldTransform().getOrigin();
    Vector3f origin = Vector3f(tmpOrigin.x(), tmpOrigin.y(), tmpOrigin.z());
    restPosition = origin;
    
    if(m_rigidBody->isActive() && (!m_rigidBody->isStaticObject()))
    {
        if(m_rigidBody->getCollisionShape()->getShapeType() == BOX_SHAPE_PROXYTYPE)
        {
            btBoxShape *box = (btBoxShape *)m_rigidBody->getCollisionShape();
            
            btVector3 halfExtents = box->getHalfExtentsWithoutMargin();
            
            Vector3f fullExtent = Vector3f(halfExtents.x(), halfExtents.y(), halfExtents.z()) * 2;
            Vector3f offset = fullExtent / (density + 1);
            
            // TopSide
            for(float x = 0; x <= fullExtent.x(); x+=offset.x())
            {
                for (float z = 0; z <= fullExtent.z(); z+=offset.z())
                {
                    Particle *p = new Particle();
                    p->mass = 0.01f;
                    p->radii = Vector3f(0.2f, 0.1f, 0.1f);
                    p->relativePosition = Vector3f(-halfExtents.x()+x, halfExtents.y(), -halfExtents.z()+z);
                    p->position = origin + p->relativePosition;
                    p->orientation = AngleAxis<float>(0, Vector3f(1, 0, 0));
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
                    p->radii = Vector3f(0.2f, 0.1f, 0.1f);
                    p->relativePosition = Vector3f(-halfExtents.x(), -halfExtents.y()+y, -halfExtents.z()+z);
                    p->position = origin + p->relativePosition;
                    p->orientation = AngleAxis<float>(0, Vector3f(1, 0, 0));
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
                    p->radii = Vector3f(0.2f, 0.1f, 0.1f);
                    p->relativePosition = Vector3f(-halfExtents.x()+x, -halfExtents.y()+y, halfExtents.z());
                    p->position = origin + p->relativePosition;
                    p->orientation = AngleAxis<float>(0, Vector3f(1, 0, 0));
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
                    p->radii = Vector3f(0.2f, 0.1f, 0.1f);
                    p->relativePosition = Vector3f(halfExtents.x(), -halfExtents.y()+y, -halfExtents.z()+z);
                    p->position = origin + p->relativePosition;
                    p->orientation = AngleAxis<float>(0, Vector3f(1, 0, 0));
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
                    p->radii = Vector3f(0.2f, 0.1f, 0.1f);
                    p->relativePosition = Vector3f(-halfExtents.x()+x, -halfExtents.y()+y, -halfExtents.z());
                    p->position = origin + p->relativePosition;
                    p->orientation = AngleAxis<float>(0, Vector3f(1, 0, 0));
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
                    p->radii = Vector3f(0.2f, 0.1f, 0.1f);
                    p->relativePosition = Vector3f(-halfExtents.x()+x, -halfExtents.y(), -halfExtents.z()+z);
                    p->position = origin + p->relativePosition;
                    p->orientation = AngleAxis<float>(0, Vector3f(1, 0, 0));
                    addAttachedParticle(p);
                }
            }
            
            // create inner particle groups
            vector<Particle*>::iterator p;
            for(p = m_attachedParticles.begin(); p!= m_attachedParticles.end(); p++)
            {
                Particle *currentCenterParticle = *p;
                
                // sort all particles by distance to current center
                vector<Particle*> sorted;
                vector<Particle*>::iterator p2;
                for(p2 = m_attachedParticles.begin(); p2 != m_attachedParticles.end(); p2++)
                {
                    float distanceToCenter = (currentCenterParticle->restPosition - (*p2)->restPosition).squaredNorm();
                    
                    int position = 0;
                    for(int i = 1; i <= sorted.size(); i++)
                    {
                        position = i;
                        Particle *sortedParticle = sorted[i-1];
                        if (distanceToCenter < (currentCenterParticle->restPosition - sortedParticle->restPosition).squaredNorm())
                        {
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
                    if (vectorIsEqualWithinMargin(currentCenterParticle->position, (*p2)->position / extrude, 0.1f))
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
                    float distanceToCenter = (currentCenterParticle->restPosition - (*p2)->restPosition).squaredNorm();
                    
                    int position = 0;
                    for(int i = 1; i <= sorted.size(); i++)
                    {
                        position = i;
                        Particle *sortedParticle = sorted[i-1];
                        if (distanceToCenter < (currentCenterParticle->restPosition - sortedParticle->restPosition).squaredNorm())
                        {
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
                for(p2 = m_attachedParticles.begin(); p2 != m_attachedParticles.end(); p2++)
                {
                    if (vectorIsEqualWithinMargin(currentCenterParticle->position, (*p2)->position * extrude, 0.1f))
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
        if (!vectorIsEqualWithinMargin((*p)->position, newParticle->position, 0.1f)) {
            isNew = true;
        }
        else
        {
            isNew = false;
            (*p)->orientation = (*p)->orientation * newParticle->restOrientation;
            break;
        }
    }
    
    if (isNew) {
        m_attachedParticles.push_back(newParticle);
        m_particles.push_back(newParticle);
        
        Particle *oP = new Particle();
        oP->mass = 0.01f;
        oP->radii = Vector3f(0.2f, 0.1f, 0.1f);
        btVector3 tmpOrigin = this->m_rigidBody->getWorldTransform().getOrigin();
        Vector3f origin = Vector3f(tmpOrigin.x(), tmpOrigin.y(), tmpOrigin.z());
        oP->position = origin + (newParticle->position - origin) * extrude;
        oP->orientation = AngleAxis<float>(0, Vector3f(1, 0, 0));
        addOuterParticle(oP);
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

bool CombinedBody::vectorIsEqualWithinMargin(Vector3f v1, Vector3f v2, float margin)
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
        (*p)->linearVelocity += timeStep * Vector3f(gravity.x(), gravity.y(), gravity.z());
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
        if ((*p)->angularVelocity.norm() < 0.1f)
        {
            (*p)->predictedOrientation = (*p)->orientation;
        }
        else
        {
            Vector3f temp = (*p)->angularVelocity.normalized() * sin(((*p)->angularVelocity.norm()*timeStep)/2);
            (*p)->predictedOrientation = Quaternionf(cos(((*p)->angularVelocity.norm()*timeStep)/2),temp.x(), temp.y(), temp.z());
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
            Quaternionf temp = (*p)->predictedOrientation * (*p)->orientation.inverse();
            // there are always two rotations, choose the shorter one
            if (temp.w() < 0)
            {
                temp = Quaternionf(-temp.w(), -temp.x(), -temp.y(), -temp.z());
            }
            
            float angle = Particle::angleQuaternion(temp);
            if (angle < 0.01f)
            {
                (*p)->angularVelocity = Vector3f::Zero();
            }
            else
            {
                (*p)->angularVelocity = Particle::axisQuaternion(temp) * angle / timeStep;
            }
            
            (*p)->orientation = (*p)->predictedOrientation;
        }
    }
}



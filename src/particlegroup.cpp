#include <iostream>
using namespace std;

#include "particlegroup.h"
#include <BulletDynamics/btBulletDynamicsCommon.h>

ParticleGroup::ParticleGroup() {}
ParticleGroup::~ParticleGroup() {}

void ParticleGroup::init()
{
    groupMass = 0.0f;
    restCenterOfMass = btVector3(0,0,0);
    
    vector<Particle*>::iterator p;
    for (p = m_particles.begin(); p!= m_particles.end(); p++)
    {
        Particle *particle = *p;
        particle->init();
        
        groupMass += particle->mass;
        restCenterOfMass += particle->mass * particle->restPosition;
    }
    restCenterOfMass /= groupMass;
}

void ParticleGroup::update()
{
//    cout << "particle group: " << endl;
    btVector3 currentCenterOfMass = calculateCenterOfMass();
//    cout << "mass center: " << currentCenterOfMass << endl;
    btMatrix3x3 R = polarDecomposition(calculateMomentMatrix());
//    cout << "R: " <<
//    R[0][0] << "|" << R[0][1] << "|" << R[0][2] << "|" <<
//    R[1][0] << "|" << R[1][1] << "|" << R[1][2] << "|" <<
//    R[2][0] << "|" << R[2][1] << "|" << R[2][2] << "|" << endl;
    
    btQuaternion optimalR;
    R.getRotation(optimalR);
    
//    cout << "optimal_axis: " << optimalR.getAxis().x() << "|" << optimalR.getAxis().y() << "|" << optimalR.getAxis().z() << endl;
//    cout << "optimal_angle: " << optimalR.getAngle() << endl;
    
    vector<Particle*>::iterator p;
    for (p = m_particles.begin(); p!= m_particles.end(); p++)
    {
        Particle *particle = *p;
//        cout << "particle: " << particle << endl;
        
        btVector3 goalPosition = R * (particle->restPosition - restCenterOfMass) + currentCenterOfMass;
//        cout << "old particle position: " << particle->predictedPosition.x() << "|" << particle->predictedPosition.y() << "|" << particle->predictedPosition.z() << "|" << endl;
        particle->predictedPosition += 0.1 * (goalPosition - particle->predictedPosition);
//        cout << "new particle position: " << particle->predictedPosition.x() << "|" << particle->predictedPosition.y() << "|" << particle->predictedPosition.z() << "|" << endl;
    }
    
    R.getRotation(m_particles.at(0)->predictedOrientation);
}

btVector3 ParticleGroup::calculateCenterOfMass()
{
    btVector3 centerOfMass = btVector3(0,0,0);
    
    vector<Particle*>::iterator p;
    for (p = m_particles.begin(); p!= m_particles.end(); p++)
    {
        Particle *particle = *p;
        
        centerOfMass += particle->mass * particle->predictedPosition;
    }
    
    return centerOfMass /= groupMass;
}

btMatrix3x3 ParticleGroup::calculateMomentMatrix()
{
    btMatrix3x3 moment = btMatrix3x3(0,0,0,0,0,0,0,0,0);
    
    vector<Particle*>::iterator p;
    for (p = m_particles.begin(); p!= m_particles.end(); p++)
    {
        Particle *particle = *p;
        btMatrix3x3 pMoment = particle->getMomentMatrix();
        moment += pMoment + VecTimesTranspose(particle->mass * particle->predictedPosition,particle->restPosition);
    }
    
    return moment - VecTimesTranspose(groupMass * calculateCenterOfMass(),restCenterOfMass);
}

btMatrix3x3 ParticleGroup::polarDecomposition(btMatrix3x3 moment)
{
    btMatrix3x3 q;
    btMatrix3x3 s;
    
    PolarDecompose(moment, q, s);
    
    return q;
}

//
// Polar decomposition,
// "Computing the Polar Decomposition with Applications", Nicholas J. Higham, 1986.
//
int ParticleGroup::PolarDecompose(const btMatrix3x3& m,btMatrix3x3& q,btMatrix3x3& s)
{
	static const btScalar	half=(btScalar)0.5;
	static const btScalar	accuracy=(btScalar)0.0001;
	static const int		maxiterations=16;
	int						i=0;
	btScalar				det=0;
	q	=	Mul(m,1/btVector3(m[0][0],m[1][1],m[2][2]).length());
	det	=	q.determinant();
	if(!btFuzzyZero(det))
	{
		for(;i<maxiterations;++i)
		{
			q=Mul(Add(q,Mul(q.adjoint(),1/det).transpose()),half);
			const btScalar	ndet=q.determinant();
			if(((ndet-det)*(ndet-det))>accuracy) det=ndet; else break;
		}
		/* Final orthogonalization	*/
		Orthogonalize(q);
		/* Compute 'S'				*/
		s=q.transpose()*m;
	}
	else
	{
		q.setIdentity();
		s.setIdentity();
	}
	return(i);
}

btMatrix3x3 ParticleGroup::Add(const btMatrix3x3& a, const btMatrix3x3& b)
{
	btMatrix3x3	r;
	for(int i=0;i<3;++i) r[i]=a[i]+b[i];
	return(r);
}

btMatrix3x3 ParticleGroup::Mul(const btMatrix3x3& a, btScalar b)
{
	btMatrix3x3	r;
	for(int i=0;i<3;++i) r[i]=a[i]*b;
	return(r);
}

void ParticleGroup::Orthogonalize(btMatrix3x3& m)
{
	m[2]=btCross(m[0],m[1]).normalized();
	m[1]=btCross(m[2],m[0]).normalized();
	m[0]=btCross(m[1],m[2]).normalized();
}

btMatrix3x3 ParticleGroup::VecTimesTranspose(const btVector3 &a, const btVector3 &b) {
    return btMatrix3x3(
                       a.x() * b.x(),a.x() * b.y(),a.x() * b.z(),
                       a.y() * b.x(),a.y() * b.y(),a.y() * b.z(),
                       a.z() * b.x(),a.z() * b.y(),a.z() * b.z());
}

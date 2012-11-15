#ifndef __Thesis__particlegroup__
#define __Thesis__particlegroup__

#include <vector>
using namespace std;

#include "particle.h"

class ParticleGroup
{
public:
    ParticleGroup();
    ~ParticleGroup();

    vector<Particle*> m_particles;
    float groupMass;
    btVector3 restCenterOfMass;
    
    void init();
    void update();
    
    btMatrix3x3 calculateMomentMatrix();
    btVector3 calculateCenterOfMass();
    btMatrix3x3 polarDecomposition(btMatrix3x3 moment);
	
private:
	static int PolarDecompose(const btMatrix3x3& m,btMatrix3x3& q,btMatrix3x3& s);
	static btMatrix3x3 Mul(const btMatrix3x3& a, btScalar b);
	static btMatrix3x3 Add(const btMatrix3x3& a, const btMatrix3x3& b);
	static void Orthogonalize(btMatrix3x3& m);
	static btMatrix3x3 VecTimesTranspose(const btVector3 &a, const btVector3 &b);
};

#endif
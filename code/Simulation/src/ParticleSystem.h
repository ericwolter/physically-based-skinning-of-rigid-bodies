#ifndef __PARTICLESYSTEM_H__
#define __PARTICLESYSTEM_H__

#include "Common/Config.h"
#include "Math2/Vec.h"

namespace IBDS 
{	
	class ParticleSystem
	{
		public:
			struct Spring
			{
				int index1;
				int index2;
				Real length;
				Real springConstant;
				Real friction;
			};

			static const int MAX_PARTICLES = 1600;
			static const int MAX_SPRINGS = 10000;

			ParticleSystem();
			~ParticleSystem(void);

			Real *getState();
			Real *getForces();
			Real *getMasses();
			Real *getPos(int i);
			Real *getVel(int i);
			Real *getForce(int i);
			Real *getMass(int i);
			Spring *getSpring(int i);
			int getNumberOfParticles() const;
			int getNumberOfSprings() const;
			void setNumberOfParticles(int num);
			void computeForces();
			void addSpring(const int i, const int j, const Real springConst, const Real frict);

		private:
			int numberOfParticles;
			int numberOfSprings;
			Real xv[6*MAX_PARTICLES];
			//Real x[MAX_PARTICLES*3];
			//Real v[MAX_PARTICLES*3];
			Real m[MAX_PARTICLES];
			Real f[3*MAX_PARTICLES];
			Spring springs[MAX_SPRINGS];			
	};
}

#endif // __PARTICLESYSTEM_H__
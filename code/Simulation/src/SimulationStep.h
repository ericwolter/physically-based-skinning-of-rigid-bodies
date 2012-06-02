#ifndef __SIMULATIONSTEP_H__
#define __SIMULATIONSTEP_H__

#include "Common/Config.h"
#include "ParticleSystem.h"

namespace IBDS 
{	
	class SimulationStep
	{
		public:			
			SimulationStep(ParticleSystem *ps, const Real timeStepSize, const int ode);
			~SimulationStep(void);

			void timeStep(void);
			void setTimeStepSize(Real stepSize);
			Real getTime() const;			

		private:
			Real time;
			Real h;
			ParticleSystem *ps;	
			int solver;

			void eulerStep();
			void rk4Step();
			void clearForces();
			void computeForces(Real *xv);			
			void derive(Real *state, Real *dst);
			void scale(int n, Real *src, Real factor, Real *dst);
			void addScale(int n, Real *src1, Real factor1, Real *src2, Real *dst);
	};
}

#endif // __SIMULATIONSTEP_H__
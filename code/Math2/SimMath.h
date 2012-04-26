/*
 * IBDS - Impulse-Based Dynamic Simulation Library
 * Copyright (c) 2003-2008 Jan Bender http://www.impulse-based.de
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * Jan Bender - Jan.Bender@impulse-based.de
 */

#ifndef __SIMMATH_H__
#define __SIMMATH_H__

#include "Common/Config.h"
#include "Vec.h"
#include "Mat.h"

namespace IBDS 
{
	typedef Vec<Real, 3> (*rungeKuttaFct)(const Real x, const Vec<Real, 3> &y, void *obj);

	/** Klasse für Mathematik-Hilfsfunktionen, die für 
	  * die Simulation benötigt werden.
	  */
	class SimMath
	{
	public:
		/** Epsilon wird für Tests auf 0 verwendet */
		static Real eps;
		/** Epsilon im Quadrat */
		static Real eps2;
		/** Cash-Karp-Parameter für Embedded Runge-Kutta */
		static Real	a2, a3, a4, a5, a6,
				b21, b31, b32, b41, b42, b43, b51, b52, b53, b54, b61, b62, b63, b64, b65,
				c1, c3, c4, c6, dc1, dc3, dc4, dc5, dc6;

		static const Vec<Real, 3> lotpunkt(const Vec<Real, 3> &p, const Vec<Real, 3> &v, const Vec<Real, 3> &s);
		static Mat<Real, 3, 3> rotationsmatrix (const Vec<Real, 3> &a, const Real phi);
		static Mat<Real, 3, 3> orthonormalize (const Mat<Real, 3, 3> &M);
		static Vec<Real, 3> computeBoxIntertiaTensor (const Real m, const Real x, const Real y, const Real z);
		static Vec<Real, 3> computeSphereIntertiaTensor (const Real m, const Real r);
		static int n_over_k (const int n, const int k);
		static Vec<Real, 3> *localCoordinates (Mat<Real, 3, 3> *rotationMatrix, Vec<Real, 3> *centerOfMass, Vec<Real, 3> *point);
		static Mat<Real, 3, 3> crossProductMatrix (const Vec<Real, 3> &r);

		static Vec<Real, 3> getEulerAngles (Mat<Real, 3, 3> *m);
		static Vec<Real, 3> rungeKutta (rungeKuttaFct f, const Real h, const Real xn, const Vec<Real, 3> &yn, void *obj, bool &result);
		static Vec<Real, 3> rungeKutta (rungeKuttaFct f, const Real h, const Vec<Real, 3> &yn, void *obj, bool &result);
		static Vec<Real, 3> embeddedRungeKutta (rungeKuttaFct f, const Real h, const Real xn, const Vec<Real, 3> &yn, Real &error, void *obj, bool &result);
		static Vec<Real, 3> embeddedRungeKutta (rungeKuttaFct f, const Real h, const Vec<Real, 3> &yn, Real &error, void *obj, bool &result);
		static Real computeVolume(const Vec<Real, 3> &a, const Vec<Real, 3> &b, const Vec<Real, 3> &c, const Vec<Real, 3> &d);
		static Real computeVolumeOriented(const Vec<Real, 3> &a, const Vec<Real, 3> &b, const Vec<Real, 3> &c, const Vec<Real, 3> &d);
	};
}

#endif

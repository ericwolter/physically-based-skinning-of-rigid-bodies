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

#ifndef __CDMATH_H__
#define __CDMATH_H__

#include <Common/Config.h>
#include "Vec.h"
#include "MathDefs.h"

namespace IBDS 
{
	/** CDMath enthält einige wichtige mathematischen Funktionen,
	  * die bei der Kollisionserkennung benötigt werden.
	  \author Jan Bender
	  */
	class CDMath
	{
	public:
		static bool sameSide (const Vec<Real,3> &p1, const Vec<Real,3> &p2, const Vec<Real,3> &a, const Vec<Real,3> &b);  
		static bool pointInTriangle(const Vec<Real,3> &p, const Vec<Real,3> &a, const Vec<Real,3> &b, const Vec<Real,3> &c);
		static Real intersectionEdgeTriangle (const Vec<Real,3> &e1, const Vec<Real,3> &e2, const Vec<Real,3> &a, const Vec<Real,3> &b, const Vec<Real,3> &c, HesseNormalForm &hnf, Vec<Real,3> &iPoint);
		static bool distancePointEdge (const Vec<Real,3> &p, const Vec<Real,3> &e1, const Vec<Real,3> &e2, Real & dist, Vec<Real,3> &l);
		static bool distancePointEdge2 (const Vec<Real,3> &p, const Vec<Real,3> &e1, const Vec<Real,3> &e2, Real & dist, Vec<Real,3> &l);
		static Real distanceEdgeEdge (const Vec<Real,3> &a1, const Vec<Real,3> &b1, const Vec<Real,3> &a2, const Vec<Real,3> &b2, Vec<Real,3> &np1, Vec<Real,3> &np2, bool &vv);
		static int lineSegmentIntersection2D (const Vec<Real,2> &a, const Vec<Real,2> &b, const Vec<Real,2> &c, const Vec<Real,2> &d, Vec<Real,2> &intersection);
	};
}

#endif

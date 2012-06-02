/*
 * IBDS - Impulse-Based Dynamic Simulation Library
 * Copyright (c) 2003-2010 Jan Bender http://www.impulse-based.de
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

#ifndef __MATH_QUATERNION_H__
#define __MATH_QUATERNION_H__

#include "Common/Config.h"
#include <iostream>
#include "Vec.h"
#include "Mat.h"


namespace IBDS 
{
	/** Quaternion is a class for calculations with quaternions.
	  * The quaternion is a vector (w, x, y, z).
	  \author Jan Bender
	  */
	class Quaternion : public VecBase<Real, 4>
	{
	public:
		/** Standard constructor
		 */
		FORCE_INLINE Quaternion() : VecBase<Real, 4>()
		{
		}
		
		/** This constructor can initialize the matrix.
		 */
		FORCE_INLINE Quaternion(bool initialize) : VecBase<Real, 4>(initialize)
		{
		}

		/** Copy constructor 
		 */
		FORCE_INLINE Quaternion(const Quaternion& q)  : VecBase<Real, 4>(q)
		{
		}

		/** Conversion
		 */
		FORCE_INLINE Quaternion(const VecBase& v)  : VecBase<Real, 4>(v)
		{
		}		

		FORCE_INLINE Quaternion(const Real &w, const Real &x, const Real &y, const Real &z) 
		{
			elem[0] = w;
			elem[1] = x;
			elem[2] = y;
			elem[3] = z;
		} 

		using VecBase<Real, 4>::operator =;
		using VecBase<Real, 4>::operator ();
		using VecBase<Real, 4>::operator [];

		void setFromAxisAngle (const Vec<Real, 3> &axis, const Real &angle);
		void getAxisAngle (Vec<Real, 3> &axis, Real &angle) const;
		void setFromMatrix3x3 (const Mat<Real, 3, 3> &m);
		void setFromMatrix3x3T (const Mat<Real, 3, 3> &m);
		void getMatrix3x3 (Mat<Real, 3, 3> &m) const;
		void getMatrix3x3T (Mat<Real, 3, 3> &m) const;
		Quaternion conjugate () const;
	};

	/** Multiplication: v * q\n
	 * [0, v]*q
	 */
	FORCE_INLINE const Quaternion operator | (const Quaternion& a, const Vec<Real, 3>& b);

	/** Multiplication: q1 * q2\n
	 * [w1, v1]*[w2,v2]= [w1*w2 - v1*v2, w1*v2 + w2*v1 + v1^v2] with v={x,y,z}
	 */
	FORCE_INLINE const Quaternion operator * (const Quaternion& a, const Quaternion& b);

	/** Multiplication: v * q\n
	 * [0, v]*q
	 */
	FORCE_INLINE const Quaternion operator | (const Vec<Real, 3>& a, const Quaternion& b);

	/** Addition
	 */
	FORCE_INLINE const Quaternion operator + (const Quaternion& q1, const Quaternion& q2);

	/** Substraction
	 */
	FORCE_INLINE const Quaternion operator - (const Quaternion& q1, const Quaternion& q2);

	/** Multiplication with a scalar
	  */
	FORCE_INLINE const Quaternion operator * (const Quaternion& q, const Real& s);

	/** Multiplication with a scalar
	 */
	FORCE_INLINE const Quaternion operator * (const Real& s, const Quaternion& q);

	/** Negation
	 */
	FORCE_INLINE const Quaternion operator - (const Quaternion& q);
}

#endif


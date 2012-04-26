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

#include "Quaternion.h"
#include "SimMath.h"
#include <math.h>

using namespace IBDS;


/** Convert a rotation around an axis in a quaternion.
  * quaternion = (cos (angle/2), sin(angle/2) * axis)
  */
void Quaternion::setFromAxisAngle (const Vec<Real, 3> &axis, const Real &angle)
{
	Real a = 0.5 * angle;
	elem[0] = cos (a);

	Real sinus = sin (a);
	elem[1] = sinus * axis[0];
	elem[2] = sinus * axis[1];
	elem[3] = sinus * axis[2];
}

/** Return the rotation axis and angle of this quaternion.
 */
void Quaternion::getAxisAngle (Vec<Real, 3> &axis, Real &angle) const
{
  	const Real l2 = elem[1]*elem[1] + elem[2]*elem[2] + elem[3]*elem[3];
	if (l2 > SimMath::eps2)
	{
		Real w = elem[0];
		if (w > 1.0)
			w = 1.0;
		else if (w < -1.0) 
			w = -1.0;
		angle = (Real) 2.0 * acos (w);
		const Real l = sqrt (l2);
		axis[0] = elem[1] / l;
		axis[1] = elem[2] / l;
		axis[2] = elem[3] / l;
	}
	else
	{
		angle = 0.0;
		axis[0] = 1.0;
		axis[1] = 0.0;
		axis[2] = 0.0;
	}
}


/** Convert a 3x3 rotation matrix 
 * in a quaternion. \n
 * (algorithm: see Ken Shoemake (SIGGRAPH))
 */
void Quaternion::setFromMatrix3x3 (const Mat<Real, 3, 3> &m)
{
	Real tr = 1.0 + m[0][0] + m[1][1] + m[2][2];
	Real s;

	if (tr > EPSILON)
	{
		s = sqrt (tr);
		elem[0] = 0.5*s;
		s = 0.5 /s;
		elem[1] = (m[2][1] - m[1][2]) * s;
		elem[2] = (m[0][2] - m[2][0]) * s;
		elem[3] = (m[1][0] - m[0][1]) * s;
	}
	else
	{
		int i = 0;
		if (m[1][1] > m[0][0])
			i = 1;
		if (m[2][2] > m[i][i])
			i = 2;
		
		switch (i)
		{
			case 0: 
					s = sqrt ((m[0][0] - (m[1][1] + m[2][2])) + 1);
					elem[1] = 0.5 * s;
					s = 0.5 / s;
					elem[2] = (m[0][1] + m[1][0]) * s;
					elem[3] = (m[2][0] + m[0][2]) * s;
					elem[0] = (m[2][1] - m[1][2]) * s;
					break;
			case 1:
					s = sqrt ((m[1][1] - (m[2][2] + m[0][0])) + 1);
					elem[2] = 0.5 * s;
					s = 0.5 / s;
					elem[3] = (m[1][2] + m[2][1]) * s;
					elem[1] = (m[0][1] + m[1][0]) * s;
					elem[0] = (m[0][2] - m[2][0]) * s;
					break;
			case 2:
					s = sqrt ((m[2][2] - (m[0][0] + m[1][1])) + 1);
					elem[3] = 0.5 * s;
					s = 0.5 / s;
					elem[1]= (m[2][0] + m[0][2]) * s;
					elem[2] = (m[1][2] + m[2][1]) * s;
					elem[0] = (m[1][0] - m[0][1]) * s;
					break;
		}
	}
}


/** Convert a tranposed 3x3 rotation matrix in a quaternion.
 * (algorithm: see Ken Shoemake (SIGGRAPH))
 */
void Quaternion::setFromMatrix3x3T (const Mat<Real, 3, 3> &m)
{
	Real tr = 1.0 + m[0][0] + m[1][1] + m[2][2];
	Real s;

	if (tr > EPSILON)
	{
		s = sqrt (tr);
		elem[0] = 0.5*s;
		s = 0.5 /s;
		elem[1] = (m[1][2] - m[2][1]) * s;
		elem[2] = (m[2][0] - m[0][2]) * s;
		elem[3] = (m[0][1] - m[1][0]) * s;
	}
	else
	{
		int i = 0;
		if (m[1][1] > m[0][0])
			i = 1;
		if (m[2][2] > m[i][i])
			i = 2;
		
		switch (i)
		{
			case 0: 
					s = sqrt ((m[0][0] - (m[1][1] + m[2][2])) + 1);
					elem[1] = 0.5 * s;
					s = 0.5 / s;
					elem[2] = (m[1][0] + m[0][1]) * s;
					elem[3] = (m[0][2] + m[2][0]) * s;
					elem[0] = (m[1][2] - m[2][1]) * s;
					break;
			case 1:
					s = sqrt ((m[1][1] - (m[2][2] + m[0][0])) + 1);
					elem[2] = 0.5 * s;
					s = 0.5 / s;
					elem[3] = (m[2][1] + m[1][2]) * s;
					elem[1] = (m[1][0] + m[0][1]) * s;
					elem[0] = (m[2][0] - m[0][2]) * s;
					break;
			case 2:
					s = sqrt ((m[2][2] - (m[0][0] + m[1][1])) + 1);
					elem[3] = 0.5 * s;
					s = 0.5 / s;
					elem[1]= (m[0][2] + m[2][0]) * s;
					elem[2] = (m[2][1] + m[1][2]) * s;
					elem[0] = (m[0][1] - m[1][0]) * s;
					break;
		}
	}
}

/** Convert the quaternion in a rotation matrix. 
  */
void Quaternion::getMatrix3x3 (Mat<Real, 3, 3> &m) const
{
	const Real xx = elem[1]*elem[1];
	const Real yy = elem[2]*elem[2];
	const Real zz = elem[3]*elem[3];
	const Real xy = elem[1]*elem[2];
	const Real wz = elem[0]*elem[3];
	const Real xz = elem[1]*elem[3];
	const Real wy = elem[0]*elem[2];
	const Real yz = elem[2]*elem[3];
	const Real wx = elem[0]*elem[1];
	
	m[0][1] = 2.0*(xy-wz);
	m[0][2] = 2.0*(xz+wy);

	m[1][0] = 2.0*(xy+wz);
	m[1][2] = 2.0*(yz-wx);

	m[2][0] = 2.0*(xz-wy);
	m[2][1] = 2.0*(yz+wx);

	// [Besl, McKay 1992]
	const Real ww = elem[0]*elem[0];
	m[0][0] = ww+xx-yy-zz;
	m[1][1] = ww+yy-xx-zz;
	m[2][2] = ww+zz-xx-yy;
}

/** Convert the quaternion in a transposed rotation matrix. 
  */
void Quaternion::getMatrix3x3T (Mat<Real, 3, 3> &m) const 
{
	const Real xx = elem[1]*elem[1];
	const Real yy = elem[2]*elem[2];
	const Real zz = elem[3]*elem[3];
	const Real xy = elem[1]*elem[2];
	const Real wz = elem[0]*elem[3];
	const Real xz = elem[1]*elem[3];
	const Real wy = elem[0]*elem[2];
	const Real yz = elem[2]*elem[3];
	const Real wx = elem[0]*elem[1];
	
	m[1][0] = 2.0*(xy-wz);
	m[2][0] = 2.0*(xz+wy);

	m[0][1] = 2.0*(xy+wz);
	m[2][1] = 2.0*(yz-wx);

	m[0][2] = 2.0*(xz-wy);
	m[1][2] = 2.0*(yz+wx);

	// [Besl, McKay 1992]
	const Real ww = elem[0]*elem[0];
	m[0][0] = ww+xx-yy-zz;
	m[1][1] = ww+yy-xx-zz;
	m[2][2] = ww+zz-xx-yy;

}


/** Return the conjugate quaternion: (w, -x, -y, -z)
  */
Quaternion Quaternion::conjugate () const 
{
	return Quaternion(elem[0], -elem[1], -elem[2], -elem[3]);
}



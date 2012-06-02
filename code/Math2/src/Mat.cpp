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

#include <Math2/Mat.h>
#include <Math2/Vec.h>

using namespace IBDS;

template Mat<Real, 3, 1>;
template Mat<Real, 1, 3>;

/** Casting
 */
template<typename T>
FORCE_INLINE Mat<T, 3, 1>::operator Vec<T, 3> ()
{
	return Vec<T, 3>(this->v[0][0], this->v[1][0], this->v[2][0]);
}

/** Casting
 */
template<typename T>
FORCE_INLINE Mat<T, 1, 3>::operator Vec<T, 3> ()
{
	return Vec<T, 3>(this->v[0][0], this->v[0][1], this->v[0][2]);
}
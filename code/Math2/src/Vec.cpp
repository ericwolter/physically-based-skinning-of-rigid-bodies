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

#include <Math2/Vec.h>
#include <Math2/Mat.h>

using namespace IBDS;

template Vec<Real, 3>;

/** Casting
 */
template<typename T>
FORCE_INLINE IBDS::Vec<T, 3>::operator Mat<T, 3, 1> ()
{
	return Mat<T, 3, 1>(this->elem[0], this->elem[1], this->elem[2]);
}

/** Casting
 */
template<typename T>
FORCE_INLINE IBDS::Vec<T, 3>::operator Mat<T, 1, 3> ()
{
	return Mat<T, 1, 3>(this->elem[0], this->elem[1], this->elem[2]);
}
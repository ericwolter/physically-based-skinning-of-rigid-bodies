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

#ifndef __MATHDEFS_H__
#define __MATHDEFS_H__

#include "Common/Config.h"


/** Epsilon wird für Tests gegen 0 verwendet (=> Rundungsfehler)
  */
#define EPSILON 1.0E-9

namespace IBDS 
{
	/** Ebene in Hesse-Normalform: a*x + b*y + c*z + d = 0
	\author Jan Bender
	*/
	struct HesseNormalForm
	{
		/** Konstruktor */
		HesseNormalForm (const HesseNormalForm &h) : a(h.a), b(h.b), c(h.c), d(h.d) {};
		/** Konstruktor */
		HesseNormalForm (Real pa, Real pb, Real pc, Real pd) : a (pa), b (pb), c (pc), d (pd) {};
		/** Parameter der hessischen Normalform */
		Real a,b,c,d;
	};
}

#endif


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

#ifndef __CONFIG_H__
#define __CONFIG_H__

// Enable memory leak detection
#ifdef _DEBUG
	#define _CRTDBG_MAP_ALLOC 
	#include <stdlib.h> 
	#include <crtdbg.h> 
	#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__) 	
	#define REPORT_MEMORY_LEAKS _CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
#else
	#define REPORT_MEMORY_LEAKS
#endif

#define USE_TIMESTEP_TIMING(x) x
#ifndef USE_TIMESTEP_TIMING
#define USE_TIMESTEP_TIMING(x)
#endif

/** Real-Typ, der verwendet wird.
  */
#ifdef USE_DOUBLES
	typedef double Real;

	#define REAL_MAX DBL_MAX
	#define REAL_MIN DBL_MIN
#else
	typedef float Real;

	#define REAL_MAX FLT_MAX
	#define REAL_MIN FLT_MIN
#endif

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#ifndef	_CRT_SECURE_NO_DEPRECATE
#define _CRT_SECURE_NO_DEPRECATE
#endif

#define FORCE_INLINE __forceinline

#endif

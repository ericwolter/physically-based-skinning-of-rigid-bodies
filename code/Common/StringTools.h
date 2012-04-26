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

#ifndef __STRINGTOOLS_H__
#define __STRINGTOOLS_H__

#include "Common/Config.h"
#include <string>
#include "Math2/Vec.h"
#include "Math2/Mat.h"

namespace IBDS
{
	// String + Integer
	std::string operator + (const std::string str, const int i);
	// Integer + String
	std::string operator + (const int i, const std::string str);

	// String + Unsigned Integer
	std::string operator + (const std::string str, const unsigned int i);
	// Unsigned Integer + String
	std::string operator + (const unsigned int i, const std::string str);

	// String + Double
	std::string operator + (const std::string str, const double d);
	// Double + String
	std::string operator + (const double d, const std::string str);

	// String + Float
	std::string operator + (const std::string str, const float i);
	// Float + String
	std::string operator + (const float i, const std::string str);

	// String + Vector3D
	std::string operator + (const std::string str, const Vector3D &v);
	// Vector3D + String
	std::string operator + (const Vector3D &v, const std::string str);

	// Konvertierungen
	std::string str(unsigned int i);
	std::string str(unsigned long i);
	std::string str(int i);
	std::string str(double d);
	std::string str(bool b);
	std::string str(const Vector3D &v);
	std::string str(const Matrix3x3& m);
	std::string str(unsigned int i1, unsigned int i2, unsigned int i3);
	std::string toUpper(const std::string &str);	
	bool endsWith(const std::string &str, const std::string &ending);
}


#endif



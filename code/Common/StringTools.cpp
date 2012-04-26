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

#include "StringTools.h"
#include <sstream>
#include <ctype.h>
#include <stdio.h>

using namespace IBDS;
using namespace std;

/** Hängt ein Integer an einen String an.
  */
string IBDS::operator + (const string str, const int i)
{
   ostringstream oss;
   oss << i;
   return str + oss.str();
}

/** Fügt ein Integer an den Anfang eines String ein.
  */
string IBDS::operator + (const int i, const string str)
{
   ostringstream oss;
   oss << i;
   return oss.str() + str;
}

/** Hängt ein Integer an einen String an.
  */
string IBDS::operator + (const string str, const unsigned int i)
{
   ostringstream oss;
   oss << i;
   return str + oss.str();
}

/** Fügt ein Integer an den Anfang eines String ein.
  */
string IBDS::operator + (const unsigned int i, const string str)
{
   ostringstream oss;
   oss << i;
   return oss.str() + str;
}

/** Hängt ein Double an einen String an.
  */
string IBDS::operator + (const string str, const double d)
{
   ostringstream oss;
   oss << d;
   return str + oss.str();
}

/** Fügt ein Double an den Anfang eines String ein.
  */
string IBDS::operator + (const double d, const string str)
{
   ostringstream oss;
   oss << d;
   return oss.str() + str;
}

/** Hängt ein Float an einen String an.
  */
string IBDS::operator + (const string str, const float f)
{
   ostringstream oss;
   oss << f;
   return str + oss.str();
}

/** Fügt ein Float an den Anfang eines String ein.
  */
string IBDS::operator + (const float f, const string str)
{
   ostringstream oss;
   oss << f;
   return oss.str() + str;
}

/** Hängt ein Vector3D an einen String an.
  */
string IBDS::operator + (const string str, const Vector3D &v)
{
   ostringstream oss;
   oss << v;
   return str + oss.str();
}

/** Fügt ein Vector3D an den Anfang eines String ein.
  */
string IBDS::operator + (const Vector3D &v, const string str)
{
   ostringstream oss;
   oss << v;
   return oss.str() + str;
}

/** Konvertiert ein Double in einen String.
  */
string IBDS::str(double d)
{
   ostringstream oss;
   char s[100];
   sprintf(s,"%10.30f",d); // hack to prevent rounding
   oss << s;
   return oss.str();
}

/** Konvertiert ein Bool in einen String.
  */
string IBDS::str(bool b)
{
   ostringstream oss;
   oss << b;
   return oss.str();
}

/** Konvertiert einen Vektor in einen String.
  */
string IBDS::str(const Vector3D &v)
{
	return str(v[0]) + " " + v[1] + " " + v[2];
}

string IBDS::str(const Matrix3x3& m)
{
	return str(m[0][0]) + " " + m[0][1] + " " + m[0][2] + 
		" " + m[1][0] + " " + m[1][1] + " " + m[1][2] +
		" " + m[2][0] + " " + m[2][1] + " " + m[2][2];
}

string IBDS::str(unsigned int i1, unsigned int i2, unsigned int i3)
{
	return str(i1) + " " + str(i2) + " " + str(i3);
}

string IBDS::str( int i )
{
	ostringstream oss;
	oss << i;
	return oss.str();
}

string IBDS::str( unsigned int i )
{
	ostringstream oss;
	oss << i;
	return oss.str();
}

string IBDS::str( unsigned long i )
{
	ostringstream oss;
	oss << i;
	return oss.str();
}

string IBDS::toUpper(const string &str)
{
	string result = str;
	const unsigned int length = (unsigned int) str.length();
	for(unsigned int i=0; i < length; i++)
	{
		result[i] = toupper(str[i]);
	}
	return result;
}

bool IBDS::endsWith(const string &str, const string &ending)
{
	if (str.length() > ending.length()) 
	{
		return (0 == str.compare (str.length() - ending.length(), ending.length(), ending));
	} 
	else 
	{
		return false;
	}
}

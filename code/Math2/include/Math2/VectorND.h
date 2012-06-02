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

#ifndef __VECTORND_H__
#define __VECTORND_H__

#include "Common/Config.h"
#include "MatBase.h"
#include <iostream>

using namespace std;

namespace IBDS 
{
	class MatrixNxM;

	/** VectorND ist eine Klasse für Vektorberechnungen, wie z.B. Addition, Multiplikation,...
	  * Wegen der Geschwindigkeit werden keine Dimensionen überprüft.
	  \author Jan Bender
	  */
	class VectorND : public Matrix
	{
	public:
		/** Komponenten des Vektors */
		Real v[6];
		/* Größe des Vektors */
		int n;

	public:
		VectorND();
		VectorND(const int dim);
		~VectorND();

		friend VectorND operator - (const VectorND& v);						  // -v1
		friend VectorND operator + (const VectorND& a, const VectorND& b);	  // v1 + v2
		friend VectorND operator - (const VectorND& a, const VectorND& b);	  // v1 - v2
		friend VectorND operator * (const VectorND& a, const Real d);		  // v1 * 3.0
		friend VectorND operator * (const Real d, const VectorND& a);		  // 3.0 * v1
		friend Real operator * (const VectorND& a, const VectorND& b);	  // Skalarprodukt
		friend VectorND operator * (const VectorND& v, const MatrixNxM& m);	  // v * M
		friend VectorND operator * (const MatrixNxM& m, const VectorND& v);	  // M * v
		friend bool operator == (const VectorND &a, const VectorND &b);		  // a == b ?
		friend bool operator != (const VectorND &a, const VectorND &b);		  // a == b ?


		VectorND& operator = ( const VectorND& v );					// Zuweisung
		VectorND& operator -= (const VectorND& v);					// Subtraktion
		VectorND& operator += (const VectorND& v);					// Addition
		VectorND& operator *= ( const Real d );					// Multiplikation mit einer Konstanten 
		VectorND& operator /= ( const Real d );					// Division mit einer Konstanten
		Real& operator [] (int i);						// Zugriff per Index
		const Real& operator [] (int i) const;
		Real& operator () (int i, int j);					// Zugriff per Index
		const Real& operator () (int i, int j) const;

		friend ostream& operator << (ostream& s, const VectorND& v);	    // Streamausgabe

		Real length() const;			    // Länge
		Real length2() const;			    // Quadratische Länge
		void normalize();				// Normierung 
		Real distance (VectorND a) const;   // Abstand zum Punkt a
		Real distance2 (VectorND a) const;  // Quadratischer Abstand zum Punkt a

		int getRows () const;
		int getCols () const;
	};
}

#endif


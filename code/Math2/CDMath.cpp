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

#include "CDMath.h"
#include <math.h>

using namespace IBDS;

/** Gibt zurück, ob sich die Punkte p1, p2 auf der gleichen Seite der Linie von a nach b befinden.
  */
bool CDMath::sameSide (const Vec<Real,3> &p1, const Vec<Real,3> &p2, const Vec<Real,3> &a, const Vec<Real,3> &b)
{
	const Vec<Real,3> cp1 = (b-a) ^ (p1-a);
	const Vec<Real,3> cp2 = (b-a) ^ (p2-a);
	if (cp1 * cp2 >= -EPSILON)
		return true;
	else
		return false;
}

/** Gibt zurück, ob sich der Punkt p innerhalb des Dreiecks a,b,c befindet. 
  */
bool CDMath::pointInTriangle(const Vec<Real,3> &p, const Vec<Real,3> &a, const Vec<Real,3> &b, const Vec<Real,3> &c)
{
	if (sameSide(p, a, b, c) && sameSide(p, b, a, c) && sameSide(p, c, a, b))
		return true;
	else
		return false;
}

/** Gibt zurück, wie weit die übergebene Kante e1, e2 in das Dreieck a,b,c eindringt. 
  * Wenn die Kante das Dreieck nicht schneidet, dann wird -1 zurückgegeben.
  * Die Ebene, in der das Dreieck liegt hat die Hessische Normalform hnf.
  * In iPoint wird ggf. der Schnittpunkt zurückgegeben.
  \param e1 Punkt 1 der Kante
  \param e2 Punkt 2 der Kante
  \param a Punkt A des Dreiecks
  \param b Punkt B des Dreiecks
  \param c Punkt C des Dreiecks
  \param hnf Hessische Normalform der Ebene, in der das Dreieck liegt
  \param iPoint Rückgabe des Schnittpunkts
  */
Real CDMath::intersectionEdgeTriangle (const Vec<Real,3> &e1, const Vec<Real,3> &e2, const Vec<Real,3> &a, const Vec<Real,3> &b, const Vec<Real,3> &c,
									   HesseNormalForm &hnf, Vec<Real,3> &iPoint)
{
	// Distanz der Punkte e1, e2 zur Dreiecksebene berechnen: a*x+b*y+c*z+d
	const Real d1 = hnf.a*e1[0] + hnf.b*e1[1] + hnf.c*e1[2] + hnf.d;
	const Real d2 = hnf.a*e2[0] + hnf.b*e2[1] + hnf.c*e2[2] + hnf.d;

	// Nur bei unterschiedlichen Vorzeichen ist Schnitt möglich.
	if (((d1 < 0) && (d2 < 0)) || ((d1 > 0) && (d2 > 0)))
		return -1;

	// Bei d1=0 bzw. d2=0 liegen e1 bzw. e2 in der Ebene und bilden den Schnittpunkt
	if (d1==0) 
	{
		if (pointInTriangle (e1, a, b, c))
		{
			iPoint = e1;
			return 0;
		}
	}
	if (d2==0)
	{
		if (pointInTriangle (e2, a, b, c))
		{
			iPoint = e2;
			return 0;
		}
	}

	// Schnittpunkt der Geraden mit der Ebene bestimmen
	const Vec<Real, 3> v = e2 - e1;			// Richtungsvektor der Geraden, als Aufpunkt dient e1
	const Real div = hnf.a*v[0] + hnf.b*v[1] + hnf.c*v[2];

	// Division durch 0 verhindern: In diesem Fall wäre die Gerade
	// parallel zur Ebene und würde diese damit nicht schneiden.
	if (div == 0)
		return -1;

	const Real lambda = -d1 / div;

	// Schnittpunkt mit der Ebene
	const Vec<Real, 3> ip = e1 + lambda*v;

	// Bestimmen, ob Schnittpunkt in Dreieck liegt
	if (pointInTriangle (ip, a, b, c))
	{
		if (d1 < 0)
			return -d1;
		else
			return -d2;
		iPoint = ip;
	}
	else
		return -1;
}

/** Bestimmt den Abstand der Kante e1, e2 zu dem Punkt p.
  * Berechnet zunächst den Lotpunkt l auf der Kante. Falls
  * der Lotpunkt nicht zwischen den Punkten e1 und e2 liegt,
  * dann wird kein Abstand zurückgegeben und die Funktion
  * gibt false zurück.
  \param p Punkt für den der Abstand bestimmt werden soll
  \param e1 Erster Eckpunkt der Kante
  \param e2 Zweiter Eckpunkt der Kante
  \param dist Rückgabe des Abstands
  \param l Rückgabe des Lotpunktes
  */
bool CDMath::distancePointEdge (const Vec<Real,3> &p, const Vec<Real,3> &e1, const Vec<Real,3> &e2, Real & dist, Vec<Real,3> &l)
{
	// Richtungsvektor der Kante
	const Vec<Real,3> v = e2 - e1;
	
	// Lotpunkt bestimen
	const Real v2 = v*v;
	if (v2 == 0)
		return false;

	const Real lambda = ((p*v - v*e1) / v2);
	// Überprüfen, ob Lotpunkt zwischen e1 und e2 liegt.
	if ((lambda >= 0) && (lambda <= 1))
	{
		// Lotpunkt
		l = e1 + lambda * v;
		dist = (p - l).length ();
		return true;
	}
	else
		return false;
}

/** Bestimmt den quadratischen Abstand der Kante e1, e2 zu dem
  * Punkt p. Berechnet zunächst den Lotpunkt l auf der Kante. 
  * Falls der Lotpunkt nicht zwischen den Punkten e1 und e2 liegt,
  * dann wird kein Abstand zurückgegeben und die Funktion gibt 
  * false zurück.
  \param p Punkt für den der Abstand bestimmt werden soll
  \param e1 Erster Eckpunkt der Kante
  \param e2 Zweiter Eckpunkt der Kante
  \param dist Rückgabe des Abstands
  \param l Rückgabe des Lotpunktes
  */
bool CDMath::distancePointEdge2 (const Vec<Real,3> &p, const Vec<Real,3> &e1, const Vec<Real,3> &e2, Real & dist, Vec<Real,3> &l)
{
	// Richtungsvektor der Kante
	const Vec<Real,3> v = e2 - e1;
	
	// Lotpunkt bestimen
	const Real v2 = v*v;
	if (v2 == 0)
		return false;

	const Real lambda = ((p*v - v*e1) / v2);
	// Überprüfen, ob Lotpunkt zwischen e1 und e2 liegt.
	if ((lambda >= 0) && (lambda <= 1))
	{
		// Lotpunkt
		l = e1 + lambda * v;
		dist = (p - l).length ();
		return true;
	}
	else
		return false;
}

/** Berechnet den Abstand zwischen zwei Kanten nach dem Algorithmus von V. J. Lumelsky, der 
  * in "Impulsbasierte Dynamiksimulation starrer Körper unter Verwendung von Hüllkörperhierarchien" 
  * (Christian Lennerz) ausführlich beschrieben ist.
  \param a1 erster Eckpunkt der ersten Kante
  \param b1 zweiter Eckpunkt der ersten Kante
  \param a2 erster Eckpunkt der zweiten Kante
  \param b2 zweiter Eckpunkt der zweiten Kante
  \param np1 Rückgabe des nahesten Punktes auf der ersten Kante
  \param np2 Rückgabe des nahesten Punktes auf der zweiten Kante
  \param vv Gibt true zurück, wenn es sich um einen Vertex-Vertex- oder Vertex-Edge-Fall handelt. Also, wenn sich der nächste Punkt 
		    auf einer Kante in einem Endpunkt befindet.
  */
Real CDMath::distanceEdgeEdge (const Vec<Real,3> &a1, const Vec<Real,3> &b1, const Vec<Real,3> &a2, const Vec<Real,3> &b2, Vec<Real,3> &np1, Vec<Real,3> &np2, bool &vv)
{
	const Vec<Real,3> v1 = b1-a1;
	const Vec<Real,3> v2 = b2-a2;
	const Vec<Real,3> v12 = a2-a1;
	vv = false;

	const Real V12 = v1 * v2;
	const Real V1 = v1 * v1;
	const Real V2 = v2 * v2;
	const Real W1 = v1 * v12;
	const Real W2 = v2 * v12;

	Real u1 = 0;
	Real lambda1 = 0;

	if (V1*V2-V12*V12 == 0)
	{
		vv = true;
		u1 = 0;
	}
	else
	{
		lambda1 = (W1*V2 - W2*V12) / (V1*V2 - V12*V12);
		if (lambda1 < 0)
		{
			u1 = 0;
			vv = true;
		}
		else if (lambda1 > 1)
		{
			vv = true;
			u1 = 1;
		}
		else 
			u1 = lambda1;
	}
	Real u2 = (u1*V12 - W2) / V2;
	if ((u2 >= 0) && (u2 <= 1))
	{
		np1 = a1 + u1 * v1;
		np2 = a2 + u2 * v2;
		const Vec<Real, 3> h = u1*v1 - u2*v2 - v12;
		return h*h;
	}
	else
	{
		vv = true;
		if (u2 < 0)
			u2 = 0;
		else if (u2 > 1)
			u2 = 1;	
		u1 = (u2*V12 + W1) / V1;
		if ((u1 >= 0) && (u1 <= 1))
		{
			np1 = a1 + u1 * v1;
			np2 = a2 + u2 * v2;
			const Vec<Real, 3> h = u1*v1 - u2*v2 - v12;
			return h*h;
		}
		else 
		{
			if (u1 < 0)
				u1 = 0;
			else if (u1 > 1)
				u1 = 1;
			np1 = a1 + u1 * v1;
			np2 = a2 + u2 * v2;
			const Vec<Real, 3> h = u1*v1 - u2*v2 - v12;
			return h*h;
		}
	}
}

/** Bestimmt den Schnitt von zwei Liniensegmenten (a,b) und (c,d) und gibt das Ergebnis in intersection zurück. 
  * Der Rückgabewert hat folgende Bedeutung: \n
  * 0: Es existiert kein Schnittpunkt.\n
  * 1: Es wurde ein Schnittpunkt gefunden.\n
  * 2: Segmente sind parallel.\n
  * 3: Linien liegen übereinander.
  */ 
int CDMath::lineSegmentIntersection2D (const Vec<Real,2> &a, const Vec<Real,2> &b, const Vec<Real,2> &c, const Vec<Real,2> &d, Vec<Real,2> &intersection)
{
	const Vec<Real, 2> v1 = b-a;
	const Vec<Real, 2> v2 = d-c;
	const Vec<Real, 2> delta = c-a;

	const Real denom = v1[0]*v2[1] - v1[1]*v2[0];

	const Real num1 = delta[0]*v2[1] - delta[1]*v2[0];
	const Real num2 = delta[0]*v1[1] - delta[1]*v1[0];

	// Linien parallel
	if (fabs(denom) < 1.0e-6)
	{
		if ((fabs(num1) < 1.0e-6) && (fabs(num2) < 1.0e-6))
		{
			return 3;
		}
		return 2;
	}

	const Real s = num1 / denom;
	const Real t = num2 / denom;

	if ((s >= 0) && (s <= 1) && (t >= 0) && (t <= 1))
	{
		intersection = a + s*v1;
		return 1;
	}
	return 0;
}

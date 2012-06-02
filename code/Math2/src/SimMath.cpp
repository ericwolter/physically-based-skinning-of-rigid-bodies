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

#include <Math2/SimMath.h>
#include <cmath>

using namespace IBDS;

Real SimMath::eps = 1.0E-8;
Real SimMath::eps2 = 1.0E-16;

// Cash-Karp-Parameter für Embedded Runge-Kutta initialisieren
Real SimMath::a2=0.2;
Real SimMath::a3=0.3;
Real SimMath::a4=0.6;
Real SimMath::a5=1.0;
Real SimMath::a6=0.875;
	
Real SimMath::b21=0.2;
Real SimMath::b31=3.0/40.0;
Real SimMath::b32=9.0/40.0;
Real SimMath::b41=0.3;
Real SimMath::b42=-0.9;
Real SimMath::b43=1.2;
Real SimMath::b51=-11.0/54.0;
Real SimMath::b52=2.5;
Real SimMath::b53=-70.0/27.0;
Real SimMath::b54=35.0/27.0;
Real SimMath::b61=1631.0/55296.0;
Real SimMath::b62=175.0/512.0;
Real SimMath::b63=575.0/13824;
Real SimMath::b64=44275.0/110592.0;
Real SimMath::b65=253.0/4096.0;

Real SimMath::c1=37.0/378.0;
Real SimMath::c3=250.0/621.0;
Real SimMath::c4=125.0/594.0;
Real SimMath::c6=512.0/1771.0;
	
Real SimMath::dc1 = c1 - 2825.0/27648.0;
Real SimMath::dc3 = c3 - 18575.0/48384.0;
Real SimMath::dc4 = c4 - 13525.0/55296.0;
Real SimMath::dc5 = -277.0/14336.0;
Real SimMath::dc6 = c6 - 0.25;


/** Auf der Geraden P+lambda*V, lambda=Parameter, wird der
  * Lotpunkt von S auf der Geraden bestimmt.
  * Vorsicht mit der Parameterreihenfolge!
  */
const Vec<Real, 3> SimMath::lotpunkt(const Vec<Real, 3> &p, const Vec<Real, 3> &v, const Vec<Real, 3> &s)
{
	const Real v2 = v.length2 ();
    if (v2 > eps2)
        return p + (v * (((s - p) * v) / v2));
    else
        return p;
}
 


    
/** rotationsmatrix liefert fuer den Rotationsachsenvektor A = [a,b,c]
  * und den Drehwinkel phi (radian)(Korkenzieherdrehrichtung bezueglich A)
  * eine 3x3-Rotationsmatrix, Formeln von Klimmek
  * a=Vektor nichtnormiert, phi=Rotationswinkel
  * nur noch fuer Ausgabeabbildung bei Modellausgabe benoetigt
  */
Mat<Real, 3, 3> SimMath::rotationsmatrix (const Vec<Real, 3> &a, const Real phi)
{
	Real x = a[0];
	Real y = a[1];
	Real z = a[2];
	const Real d = sqrt (x*x + y*y + z*z);
	if (d < eps)
        std::cout << "Vector of rotation matrix is zero!\n";

	x = x/d;
	y = y/d;
	z = z/d;
	// jetzt ist (x,y,z) normiert

	const Real x2 = x*x;
	const Real y2 = y*y;
	const Real z2 = z*z;
	const Real s = sin(phi);
	const Real c = cos(phi);
	const Real c1 = 1.0-c;
	const Real xyc = x*y*c1;
	const Real xzc = x*z*c1;
	const Real yzc = y*z*c1;
	const Real xs=x*s;
	const Real ys=y*s;
	const Real zs=z*s;

	return Mat<Real, 3, 3> (	Vec<Real, 3> (c + x2*c1, xyc+zs, xzc-ys),
								Vec<Real, 3> (xyc-zs, c+y2*c1, yzc+xs),
								Vec<Real, 3> (xzc+ys, yzc-xs, c+z2*c1));

}


/** crossProductMatrix liefert fuer den Vektor r
  * die Kreuzproduktmatrix.
  */
Mat<Real, 3, 3> SimMath::crossProductMatrix (const Vec<Real, 3> &r)
{
	return Mat<Real, 3, 3> (	Vec<Real, 3> (0, -r[2], r[1]),
								Vec<Real, 3> (r[2],0,-r[0]),
								Vec<Real, 3> (-r[1],r[0], 0));

}

/** Bringt kleine Abweichungen von der Orthonormalitaet
  * von M in Ordnung, ein Durchlauf genuegt, super Algorithmus!
  * Wenn bei Nutation "orthonormalize" nicht angewandt wird, so Desaster.
  */
Mat<Real, 3, 3> SimMath::orthonormalize (const Mat<Real, 3, 3> &M)
{
	Vec<Real, 3> v1 = M[0];
	Vec<Real, 3> v2 = M[1];
	Vec<Real, 3> v3 = M[2];
	v1.normalize ();
	v2.normalize ();
	v3.normalize ();
    for (int i=0; i < 10; i++)
	{
        bool stop = true;
        const Vec<Real, 3> v12 = v1 ^ v2;
		if (v3.distance2 (v12) > eps2)
            stop=false;
        const Vec<Real, 3> v31 = v3 ^ v1;
		if (v2.distance2 (v31) > eps2)
            stop=false;
        const Vec<Real, 3> v23 = v2 ^ v3;
		if (v1.distance2 (v23) > eps2)
            stop=false;
        v1 = (v1 + v23) * 0.5;
        v2 = (v2 + v31) * 0.5;
        v3 = (v3 + v12) * 0.5;
        if (stop)
            break;
	}
    return Mat<Real, 3, 3> (v1,v2,v3);
}

/** Berechnet die 3 Diagonalelemente des Trägheitstensors im Hauptachsensystem
  * von einem Quader der die Ausmasse (x, y, z) und die Masse m hat.
  */
Vec<Real, 3> SimMath::computeBoxIntertiaTensor (const Real m, const Real x, const Real y, const Real z)
{
	return Vec<Real, 3>(m*(y*y+z*z)/12.0, m*(x*x+z*z)/12.0, m*(y*y+x*x)/12.0);
}

/** Berechnet die 3 Diagonalelemente des Trägheitstensors im Hauptachsensystem
  * von einer Kugel mit dem Radius r und der Masse m.
  */
Vec<Real, 3> SimMath::computeSphereIntertiaTensor (const Real m, const Real r)
{
	const Real j = (2.0/5.0)*m*r*r;
	return Vec<Real, 3> (j,j,j);
}

/** Erzeugt einen neuen Punkt mit den lokalen Koordinaten des übergebenen Punktes. 
  */
Vec<Real, 3> *SimMath::localCoordinates (Mat<Real, 3, 3> *rotationMatrix, Vec<Real, 3> *centerOfMass, Vec<Real, 3> *point)
{
	Vec<Real, 3> *p = new Vec<Real, 3> ();
	*p = *rotationMatrix * (*point - *centerOfMass);
	return p;
}

/** Berechnet n über k. \n
  * n! / (k! * (n-k)!)
  */
int SimMath::n_over_k (int n, int k)
{
	int kf = 1;
	int nf = 1;
	int nkf = 1;
	for (int i=2; i <= n; i++)
	{
		nf *= i;
		if (i <= k)
			kf *= i;
		if (i <= n-k)
			nkf *= i;
	}
	return nf / (kf*nkf);
}



/** Berechnet aus einer 3x3 Rotationsmatrix die Eulerwinkel 
  * und gibt sie als Vektor zurück.
  */
Vec<Real, 3> SimMath::getEulerAngles (Mat<Real, 3, 3> *m)
{
	Vec<Real, 3> angles;
  
	/* Now, get the rotations out, as described in the gem. */
	Real v = -(*m)[0][2];
	if (v > 1.0)
		v = 1.0;
	else if (v < -1.0)
		v = -1.0;
	angles[1] = asin(v);
	if ( cos(angles[1]) != 0 ) 
	{
		angles[0] = atan2((*m)[1][2], (*m)[2][2]);
		angles[2] = atan2((*m)[0][1], (*m)[0][0]);
	} 
	else 
	{
		angles[0] = atan2(-(*m)[2][0], (*m)[1][1]);
		angles[2] = 0;
	}
	return angles;
}

/** Integriert eine dreidimensionale Funktion nach Runge-Kutta.
  * Es muss eine Funktion f für die Ableitung übergeben werden.
  * Wenn die erste Ableitung Null ist, dann wird die Integration
  * abgebrochen. In diesem Fall wird yn zurückgegeben und result auf true gesetzt.
  */
Vec<Real, 3> SimMath::rungeKutta (rungeKuttaFct f, const Real h, const Real xn, const Vec<Real, 3> &yn, void *obj, bool &result)
{
	const Vec<Real, 3> k1 = h * f (xn, yn, obj);

	// Wenn erste Ableitung Null ist, dann kann abgebrochen werden 
	// In diesem Fall wird yn zurückgegeben
	if (k1.length2 () < SimMath::eps2)
	{
		result = false;
		return yn;
	}
	else 
		result = true;

	const Vec<Real, 3> k2 = h * f (xn + 0.5*h, yn + 0.5*k1, obj);
	const Vec<Real, 3> k3 = h * f (xn + 0.5*h, yn + 0.5*k2, obj);
	const Vec<Real, 3> k4 = h * f (xn + h, yn + k3, obj);

	const Vec<Real, 3> ynew = yn + 1.0/6.0 * (k1 + 2.0*k2 + 2.0*k3 + k4);
	return ynew;
}

/** Integriert eine dreidimensionale Funktion nach Embedded Runge-Kutta.
  * Es muss eine Funktion f für die Ableitung übergeben werden.
  * Wenn die erste Ableitung Null ist, dann wird die Integration
  * abgebrochen. In diesem Fall wird yn zurückgegeben und result auf true gesetzt.
  */
Vec<Real, 3> SimMath::embeddedRungeKutta (rungeKuttaFct f, const Real h, const Real xn, const Vec<Real, 3> &yn, Real &error, void *obj, bool &result)
{
	const Vec<Real, 3> k1 = h * f (xn, yn, obj);

	// Wenn erste Ableitung Null ist, dann kann abgebrochen werden 
	// In diesem Fall wird yn zurückgegeben
	const Real k1l = k1.length2 ();
	if (k1l < SimMath::eps2)
	{
		error = k1l;
		result = false;
		return yn;
	}
	else 
		result = true;

	const Vec<Real, 3> k2 = h * f (xn + a2*h, yn + b21*k1, obj);
	const Vec<Real, 3> k3 = h * f (xn + a3*h, yn + b31*k1  + b32*k2, obj);
	const Vec<Real, 3> k4 = h * f (xn + a4*h, yn + b41*k1  + b42*k2 + b43*k3, obj);
	const Vec<Real, 3> k5 = h * f (xn + a5*h, yn + b51*k1  + b52*k2 + b53*k3 + b54*k4, obj);
	const Vec<Real, 3> k6 = h * f (xn + a6*h, yn + b61*k1  + b62*k2 + b63*k3 + b64*k4 + b65*k5, obj);

	const Vec<Real, 3> ynew = yn + c1*k1 + c3*k3 + c4*k4 + c6*k6;
	const Vec<Real, 3> err = dc1*k1 + dc3*k3 + dc4*k4 + dc5*k5 + dc6*k6;
	error = err.length2 ();

	return ynew;
}

/** Integriert eine dreidimensionale Funktion nach Embedded Runge-Kutta.
  * Es muss eine Funktion f für die Ableitung übergeben werden.
  * Wenn die erste Ableitung Null ist, dann wird die Integration
  * abgebrochen. In diesem Fall wird yn zurückgegeben und result auf true gesetzt.
  */
Vec<Real, 3> SimMath::embeddedRungeKutta (rungeKuttaFct f, const Real h, const Vec<Real, 3> &yn, Real &error, void *obj, bool &result)
{
	const Vec<Real, 3> k1 = h * f (0, yn, obj);

	// Wenn erste Ableitung Null ist, dann kann abgebrochen werden 
	// In diesem Fall wird yn zurückgegeben
	if (k1.length2 () < SimMath::eps2)
	{
		result = false;
		return yn;
	}
	else 
		result = true;

	const Vec<Real, 3> k2 = h * f (0, yn + b21*k1, obj);
	const Vec<Real, 3> k3 = h * f (0, yn + b31*k1  + b32*k2, obj);
	const Vec<Real, 3> k4 = h * f (0, yn + b41*k1  + b42*k2 + b43*k3, obj);
	const Vec<Real, 3> k5 = h * f (0, yn + b51*k1  + b52*k2 + b53*k3 + b54*k4, obj);
	const Vec<Real, 3> k6 = h * f (0, yn + b61*k1  + b62*k2 + b63*k3 + b64*k4 + b65*k5, obj);

	const Vec<Real, 3> ynew = yn + c1*k1 + c3*k3 + c4*k4 + c6*k6;
	const Vec<Real, 3> err = dc1*k1 + dc3*k3 + dc4*k4 + dc5*k5 + dc6*k6;
	error = err.length2 ();

	return ynew;
}

/** Compute the volume of a tetrahedron with the given points.
  */
Real SimMath::computeVolume(const Vec<Real, 3> &a, const Vec<Real, 3> &b, const Vec<Real, 3> &c, const Vec<Real, 3> &d)
{
	const Vec<Real, 3> av = a - d;
	const Vec<Real, 3> bv = b - d;
	const Vec<Real, 3> cv = c - d;
	const Real volume = (1.0/6.0) * fabs(av * (bv ^ cv));
	return volume;
}

/** Compute the volume of a tetrahedron with the given points.
*/
Real SimMath::computeVolumeOriented(const Vec<Real, 3> &a, const Vec<Real, 3> &b, const Vec<Real, 3> &c, const Vec<Real, 3> &d)
{
	Vector3D av = a - d;
	Vector3D bv = b - d;
	Vector3D cv = c - d;
	Real volume = (1.0/6.0) * (av * (bv ^ cv));
	return volume;
}
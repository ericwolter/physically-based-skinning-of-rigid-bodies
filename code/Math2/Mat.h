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

#ifndef __MAT_H__
#define __MAT_H__

#include "MatBase.h"
#include "MathDefs.h"
#include <iostream>

namespace IBDS 
{
	/** Mat is a template class for matrix operations.
	 */
	template<typename T, int rows, int cols>
	class Mat : public MatBase<T, rows, cols>
	{
	public:
		/** Standard constructor
		 */
		FORCE_INLINE Mat() : MatBase<T, rows, cols>()
		{
		}

		/** This constructor can initialize the matrix.
		 */
		FORCE_INLINE Mat(bool initialize) : MatBase<T, rows, cols>(initialize)
		{
		}

		/** This constructor initializes the matrix with the given row vectors.
		 */
		explicit FORCE_INLINE Mat(const Vec<T, cols> vec[rows])  : MatBase<T, rows, cols>(vec)
		{
		}

		/** Copy constructor 
		 */
		FORCE_INLINE Mat(const Mat<T, rows, cols>& m)  : MatBase<T, rows, cols>(m)
		{
		} 

		/** Conversion
		 */
		FORCE_INLINE Mat(const MatBase<T, rows, cols>& m)  : MatBase<T, rows, cols>(m)
		{
		} 

		using MatBase<T, rows, cols>::operator =;
		using MatBase<T, rows, cols>::operator ();
		using MatBase<T, rows, cols>::operator [];
	};

	// Matrix product
	template<typename T, int rows, int cols, int cols2>
	FORCE_INLINE Mat<T, rows, cols2> operator *(const Mat<T, rows, cols>& m1, const Mat<T, cols, cols2>& m2)
	{
		Mat<T, rows, cols2> res;
		Mat<T, rows, cols>::mult(m1, m2, res);
		return res;
	}

	/** Addition 
	 */
	template<typename T, int rows, int cols>
	FORCE_INLINE const Mat<T, rows, cols> operator +(const Mat<T, rows, cols>& m1, const Mat<T, rows, cols>& m2)
	{
		Mat<T, rows, cols> res;
		Mat<T, rows, cols>::add(m1, m2, res);
		return res;
	}

	/** Substraction
     */
	template<typename T, int rows, int cols>
	FORCE_INLINE const Mat<T, rows, cols> operator -(const Mat<T, rows, cols>& m1, const Mat<T, rows, cols>& m2)
	{
		Mat<T, rows, cols> res;
		Mat<T, rows, cols>::sub(m1, m2, res);
		return res;
	}

	/** Negation 
	 */
	template<typename T, int rows, int cols>
	FORCE_INLINE const Mat<T, rows, cols> operator - (const Mat<T, rows, cols>& m)
	{
		Mat<T, rows, cols> res;
		Mat<T, rows, cols>::neg(m, res);
		return res;
	}

	
	// Scalar product
	template<typename T, int rows, int cols>
	FORCE_INLINE const Mat<T, rows, cols> operator *(const Mat<T, rows, cols>& m, const T& s)
	{
		Mat<T, rows, cols> res;
		Mat<T, rows, cols>::mult(m, s, res);
		return res;
	}

	template<typename T, int rows, int cols>
	FORCE_INLINE const Mat<T, rows, cols> operator *(const T& s, const Mat<T, rows, cols>& m)
	{		
		Mat<T, rows, cols> res;
		Mat<T, rows, cols>::mult(m, s, res);
		return res;
	}

	// Matrix vector product
	template<typename T, int rows, int cols>
	FORCE_INLINE const Vec<T, rows> operator *(const Mat<T, rows, cols>& m, const Vec<T, cols>& v)
	{
		Vec<T, rows> res;
		Mat<T, rows, cols>::mult(m, v, res);
		return res;
	}

	template<typename T, int rows, int cols>
	FORCE_INLINE const Vec<T, cols> operator *(const Vec<T, rows>& v, const Mat<T, rows, cols>& m)
	{
		Vec<T, cols> res;
		Mat<T, rows, cols>::mult(v, m, res);
		return res;
	}

	// 
	// nxn matrix
	//

	template<typename T, int size>
	class MatNxN : public MatBase<T, size, size>
	{
	public:
		/** Standard constructor
		 */
		FORCE_INLINE MatNxN() : MatBase<T, size, size>()
		{
		}

		/** This constructor can initialize the matrix.
		 */
		FORCE_INLINE MatNxN(bool initialize) : MatBase<T, size, size>(initialize)
		{
		}

		/** This constructor initializes the matrix with the given row vectors.
		 */
		explicit FORCE_INLINE MatNxN(const Vec<T, size> vec[size])  : MatBase<T, size, size>(vec)
		{
		}

		/** Copy constructor 
		 */
		FORCE_INLINE MatNxN(const MatNxN<T, size>& m)  : MatBase<T, size, size>(m)
		{
		} 

		/** Conversion
		 */
		FORCE_INLINE MatNxN(const MatBase<T, size, size>& m)  : MatBase<T, size, size>(m)
		{
		} 

		using MatBase<T, size, size>::operator =;

		/** Set identity matrix.
	     */
		static void setIdentity(MatNxN<T, size> &m)
		{
			for (int i=0; i < size; i++)
			{
				for (int j=0; j < size; j++)
				{
					if (i==j)
						m[i][j] = 1.0;
					else
						m[i][j] = 0.0;
				}
			}
		}

		/** Set identity matrix.
	     */
		static MatNxN<T, size> getIdentity()
		{
			MatNxN<T, size> m;
			for (int i=0; i < size; i++)
			{
				for (int j=0; j < size; j++)
				{
					if (i==j)
						m[i][j] = 1.0;
					else
						m[i][j] = 0.0;
				}
			}
			return m;
		}
	};

	

	// Matrix 3x3

	template<typename T>
	class Mat<T, 3, 3> : public MatNxN<T, 3>
	{
	public:
		/** Standard constructor
		 */
		FORCE_INLINE Mat() : MatNxN<T, 3>()
		{
		}

		/** This constructor can initialize the matrix.
		 */
		FORCE_INLINE Mat(bool initialize) : MatNxN<T, 3>(initialize)
		{
		}

		/** This constructor initializes the matrix with the given row vectors.
		 */
		explicit FORCE_INLINE Mat(const Vec<T, 3> vec[3])  : MatNxN<T, 3>(vec)
		{
		}

		/** This constructor initializes the matrix with the given row vectors.
		 */
		explicit FORCE_INLINE Mat(const Vec<T, 3> &v1, const Vec<T, 3> &v2, const Vec<T, 3> &v3)
		{
			v[0] = v1;
			v[1] = v2;
			v[2] = v3;
		}

		/** Copy constructor 
		 */
		FORCE_INLINE Mat(const Mat<T, 3, 3>& m)  : MatNxN<T, 3>(m)
		{
		} 

		/** Conversion
		 */
		FORCE_INLINE Mat(const MatNxN<T, 3>& m)  : MatNxN<T, 3>(m)
		{
		} 

		using MatBase<T, 3, 3>::operator =;

		/** Matrix product
		 */
		FORCE_INLINE static void mult(const Mat<T, 3, 3>& a, const Mat<T, 3, 3>& b, Mat<T, 3, 3> &c)
		{
			c[0][0] = a[0][0]*b[0][0] + a[0][1]*b[1][0] + a[0][2]*b[2][0];
			c[0][1] = a[0][0]*b[0][1] + a[0][1]*b[1][1] + a[0][2]*b[2][1];
			c[0][2] = a[0][0]*b[0][2] + a[0][1]*b[1][2] + a[0][2]*b[2][2];

			c[1][0] = a[1][0]*b[0][0] + a[1][1]*b[1][0] + a[1][2]*b[2][0];
			c[1][1] = a[1][0]*b[0][1] + a[1][1]*b[1][1] + a[1][2]*b[2][1];
			c[1][2] = a[1][0]*b[0][2] + a[1][1]*b[1][2] + a[1][2]*b[2][2];

			c[2][0] = a[2][0]*b[0][0] + a[2][1]*b[1][0] + a[2][2]*b[2][0];
			c[2][1] = a[2][0]*b[0][1] + a[2][1]*b[1][1] + a[2][2]*b[2][1];
			c[2][2] = a[2][0]*b[0][2] + a[2][1]*b[1][2] + a[2][2]*b[2][2];
		}

		/** Matrix vector product
		 */
		FORCE_INLINE static void mult(const Mat<T, 3, 3>& m, const Vec<T, 3>& v, Vec<T, 3> &res)
		{
			res[0] = v[0]*m[0][0] + v[1]*m[0][1] + v[2]*m[0][2];
			res[1] = v[0]*m[1][0] + v[1]*m[1][1] + v[2]*m[1][2];
			res[2] = v[0]*m[2][0] + v[1]*m[2][1] + v[2]*m[2][2];
		}

		/** Vector matrix product
		 */
		FORCE_INLINE static void mult(const Vec<T, 3>& v, const Mat<T, 3, 3>& m,  Vec<T, 3> &res)
		{
			res[0] = v[0]*m[0][0] + v[1]*m[1][0] + v[2]*m[2][0];
			res[1] = v[0]*m[0][1] + v[1]*m[1][1] + v[2]*m[2][1];
			res[2] = v[0]*m[0][2] + v[1]*m[1][2] + v[2]*m[2][2];
		}

		/** Scalar product
		 */
		FORCE_INLINE static void mult(const Mat<T, 3, 3>& m, const T& s, Mat<T, 3, 3> &res)
		{
			res[0] = s * m[0];
			res[1] = s * m[1];
			res[2] = s * m[2];
		}

		/** Dyadic product
		 */
		FORCE_INLINE static void dyadic(const Vec<T, 3>& v1, const Vec<T, 3>& v2, Mat<T, 3, 3> &res)
		{
			res[0][0] = v1[0]*v2[0];
			res[0][1] = v1[0]*v2[1];
			res[0][2] = v1[0]*v2[2];
			res[1][0] = v1[1]*v2[0];
			res[1][1] = v1[1]*v2[1];
			res[1][2] = v1[1]*v2[2];
			res[2][0] = v1[2]*v2[0];
			res[2][1] = v1[2]*v2[1];
			res[2][2] = v1[2]*v2[2];
		}


		/** Returns the inverse matrix. 
		  */
		Mat<T, 3, 3> inverse () const
		{
			T a = v[0][0];
			T b = v[0][1];
			T c = v[0][2];
			T d = v[1][0];
			T e = v[1][1];
			T f = v[1][2];
			T g = v[2][0];
			T h = v[2][1];
			T i = v[2][2];
			T Div = -c*e*g+b*f*g+c*d*h-a*f*h-b*d*i+a*e*i;
			if (fabs(Div) < EPSILON)
			{
				std::cout << "matrix inversion failed\n";
				return Mat<T, 3, 3>(true);
			}
			Div = ((T) 1.0/Div);
			return Mat<T, 3, 3>(	(Vec<T, 3>(-f*h+e*i, c*h-b*i, -c*e+b*f)*Div),
									(Vec<T, 3>(f*g-d*i,-c*g+a*i,c*d-a*f)*Div),
									(Vec<T, 3>(-e*g+d*h,b*g-a*h,-b*d+a*e)*Div));
									// hat Mathematica so ermittelt
		}

		/** Gibt die Inverse einer symmetrischen, nicht singulären Matrix zurück.
		  */
		Mat<T, 3, 3> symmInverse () const
		{
			T e2_df = v[1][2]*v[1][2] - v[1][1]*v[2][2];  // e^2 - d*f
			T bf = v[0][1]*v[2][2];
			T ce = v[0][2]*v[1][2];
			T c2 = v[0][2]*v[0][2];
			T cd = v[0][2]*v[1][1];
			T be = v[0][1]*v[1][2];
			T af = v[0][0]*v[2][2];
			T ae = v[0][0]*v[1][2];
			T bc = v[0][1]*v[0][2];
			T b2 = v[0][1]*v[0][1];
			T ad = v[0][0]*v[1][1];

			T Div = (T) 1.0 / (c2*v[1][1] + v[0][1] * (bf - (T) 2.0 * ce) + v[0][0]* e2_df  );

			T a = e2_df * Div;
			T b = (bf - ce) * Div;
			T c = (cd - be) * Div;
			T d = (c2 - af) * Div;
			T e = (ae - bc) * Div;
			T f = (b2 - ad) * Div;

			return Mat<T, 3, 3>(	Vec<T, 3>(a,b,c),
									Vec<T, 3>(b,d,e),
									Vec<T, 3>(c,e,f));
		}

		/** Gibt die Determinante einer Matrix zurück.
		  */
		T det () const
		{
			T d;
			d  = v[0][0] * (v[1][1]*v[2][2] - v[1][2] * v[2][1]);		
			d -= v[0][1] * (v[1][0]*v[2][2] - v[1][2] * v[2][0]);	
			d += v[0][2] * (v[1][0]*v[2][1] - v[1][1] * v[2][0]);	
			return d;
		}

		/** Return the one norm of the matrix.
		 */
		FORCE_INLINE T oneNorm() const 
		{
			const Real sum1 = fabs(v[0][0]) + fabs(v[1][0]) + fabs(v[2][0]);
			const Real sum2 = fabs(v[0][1]) + fabs(v[1][1]) + fabs(v[2][1]);
			const Real sum3 = fabs(v[0][2]) + fabs(v[1][2]) + fabs(v[2][2]);
			Real maxSum = sum1;
			if (sum2 > maxSum)
				maxSum = sum2;
			if (sum3 > maxSum)
				maxSum = sum3;
			return maxSum;
		}

		/** Return the inf norm of the matrix.
		 */
		FORCE_INLINE T infNorm() const 
		{
			const Real sum1 = fabs(v[0][0]) + fabs(v[0][1]) + fabs(v[0][2]);
			const Real sum2 = fabs(v[1][0]) + fabs(v[1][1]) + fabs(v[1][2]);
			const Real sum3 = fabs(v[2][0]) + fabs(v[2][1]) + fabs(v[2][2]);
			Real maxSum = sum1;
			if (sum2 > maxSum)
				maxSum = sum2;
			if (sum3 > maxSum)
				maxSum = sum3;
			return maxSum;
		}

		/** Return rotation matrix (x,y,z)
		  */
		static Mat<T, 3, 3> getRotationMatrixXYZ(const T x, const T y, const T z)
		{
			const T cx = cos(x);
			const T cy = cos(y);
			const T cz = cos(z);
			const T sx = sin(x);
			const T sy = sin(y);
			const T sz = sin(z);

			return Mat<T, 3, 3>(	Vec<T, 3>(cy*cz,		cx*sz+cz*sx*sy,		sx*sz-cx*cz*sy),
									Vec<T, 3>(-cy*sz,	cx*cz-sx*sy*sz,		cx*sy*sz+cz*sx),
									Vec<T, 3>(sy,		-cy*sx,				cx*cy));
		}
	};

	// Matrix 2x2

	template<typename T>
	class Mat<T, 2, 2> : public MatNxN<T, 2>
	{
	public:
		/** Standard constructor
		 */
		FORCE_INLINE Mat() : MatNxN<T, 2>()
		{
		}

		/** This constructor can initialize the matrix.
		 */
		FORCE_INLINE Mat(bool initialize) : MatNxN<T, 2>(initialize)
		{
		}

		/** This constructor initializes the matrix with the given row vectors.
		 */
		explicit FORCE_INLINE Mat(const Vec<T, 2> vec[2])  : MatNxN<T, 2>(vec)
		{
		}

		/** This constructor initializes the matrix with the given row vectors.
		 */
		explicit FORCE_INLINE Mat(const Vec<T, 2> &v1, const Vec<T, 2> &v2)
		{
			v[0] = v1;
			v[1] = v2;
		}

		/** Copy constructor 
		 */
		FORCE_INLINE Mat(const Mat<T, 2, 2>& m)  : MatNxN<T, 2>(m)
		{
		} 

		/** Conversion
		 */
		FORCE_INLINE Mat(const MatNxN<T, 2>& m)  : MatNxN<T, 2>(m)
		{
		} 

		using MatBase<T, 2, 2>::operator =;

		/** Matrix product
		 */
		FORCE_INLINE static void mult(const Mat<T, 2, 2>& a, const Mat<T, 2, 2>& b, Mat<T, 2, 2> &c)
		{
			c[0][0] = a[0][0]*b[0][0] + a[0][1]*b[1][0];
			c[0][1] = a[0][0]*b[0][1] + a[0][1]*b[1][1];
			c[0][2] = a[0][0]*b[0][2] + a[0][1]*b[1][2];

			c[1][0] = a[1][0]*b[0][0] + a[1][1]*b[1][0];
			c[1][1] = a[1][0]*b[0][1] + a[1][1]*b[1][1];
			c[1][2] = a[1][0]*b[0][2] + a[1][1]*b[1][2];
		}

		/** Matrix vector product
		 */
		FORCE_INLINE static void mult(const Mat<T, 2, 2>& m, const Vec<T, 2>& v, Vec<T, 2> &res)
		{
			res[0] = v[0]*m[0][0] + v[1]*m[0][1];
			res[1] = v[0]*m[1][0] + v[1]*m[1][1];
		}

		/** Vector matrix product
		 */
		FORCE_INLINE static void mult(const Vec<T, 2>& v, const Mat<T, 2, 2>& m,  Vec<T, 2> &res)
		{
			res[0] = v[0]*m[0][0] + v[1]*m[1][0];
			res[1] = v[0]*m[0][1] + v[1]*m[1][1];
		}

		/** Scalar product
		 */
		FORCE_INLINE static void mult(const Mat<T, 2, 2>& m, const T& s, Mat<T, 2, 2> &res)
		{
			res[0] = s * m[0];
			res[1] = s * m[1];
		}


		/** Returns the inverse matrix. 
		  */
		Mat<T, 2, 2> inverse () const
		{
			Mat<T, 2, 2> inv;

			T a = v[0][0];
			T b = v[0][1];
			T c = v[1][0];
			T d = v[1][1];
			T Div = -b*c + a*d;
			if (fabs(Div) < EPSILON)
			{
				// print "Fehler in Matrixinvertierung"
				return Mat<T, 2, 2>(true);
			}
			return Mat<T, 2, 2> ( Vec<T, 2> (d/Div, -b/Div), Vec<T, 2> (-c/Div, a/Div));
		}

		/** Dyadic product
		 */
		FORCE_INLINE static void dyadic(const Vec<T, 2>& v1, const Vec<T, 2>& v2, Mat<T, 2, 2> &res)
		{
			res[0][0] = v1[0]*v2[0];
			res[0][1] = v1[0]*v2[1];
			res[1][0] = v1[1]*v2[0];
			res[1][1] = v1[1]*v2[1];
		}
	};


	// Matrix 3x2

	template<typename T>
	class Mat<T, 3, 2> : public MatBase<T, 3, 2>
	{
	public:
		/** Standard constructor
		 */
		FORCE_INLINE Mat() : MatBase<T, 3, 2>()
		{
		}

		/** This constructor can initialize the matrix.
		 */
		FORCE_INLINE Mat(bool initialize) : MatBase<T, 3, 2>(initialize)
		{
		}

		/** This constructor initializes the matrix with the given row vectors.
		 */
		explicit FORCE_INLINE Mat(const Vec<T, 2> vec[3])  : MatBase<T, 3, 2>(vec)
		{
		}

		/** This constructor initializes the matrix with the given row vectors.
		 */
		explicit FORCE_INLINE Mat(const Vec<T, 2> &v1, const Vec<T, 2> &v2, const Vec<T, 2> &v3)
		{
			v[0] = v1;
			v[1] = v2;
			v[2] = v3;
		}

		/** Copy constructor 
		 */
		FORCE_INLINE Mat(const Mat<T, 3, 2>& m)  : MatBase<T, 3, 2>(m)
		{
		} 

		/** Conversion
		 */
		FORCE_INLINE Mat(const MatBase<T, 3, 2>& m)  : MatBase<T, 3, 2>(m)
		{
		} 

		using MatBase<T, 3, 2>::operator =;
	};

	// Matrix 2x3

	template<typename T>
	class Mat<T, 2, 3> : public MatBase<T, 2, 3>
	{
	public:
		/** Standard constructor
		 */
		FORCE_INLINE Mat() : MatBase<T, 2, 3>()
		{
		}

		/** This constructor can initialize the matrix.
		 */
		FORCE_INLINE Mat(bool initialize) : MatBase<T, 2, 3>(initialize)
		{
		}

		/** This constructor initializes the matrix with the given row vectors.
		 */
		explicit FORCE_INLINE Mat(const Vec<T, 3> vec[2])  : MatBase<T, 2, 3>(vec)
		{
		}

		/** This constructor initializes the matrix with the given row vectors.
		 */
		explicit FORCE_INLINE Mat(const Vec<T, 3> &v1, const Vec<T, 3> &v2)
		{
			v[0] = v1;
			v[1] = v2;
		}

		/** Copy constructor 
		 */
		FORCE_INLINE Mat(const Mat<T, 2, 3>& m)  : MatBase<T, 2, 3>(m)
		{
		} 

		/** Conversion
		 */
		FORCE_INLINE Mat(const MatBase<T, 2, 3>& m)  : MatBase<T, 2, 3>(m)
		{
		} 

		using MatBase<T, 2, 3>::operator =;
	};

	// Matrix 3x1

	template<typename T>
	class Mat<T, 3, 1> : public MatBase<T, 3, 1>
	{
	public:
		/** Standard constructor
		 */
		FORCE_INLINE Mat() : MatBase<T, 3, 1>()
		{
		}

		/** This constructor can initialize the matrix.
		 */
		FORCE_INLINE Mat(bool initialize) : MatBase<T, 3, 1>(initialize)
		{
		}

		/** This constructor initializes the matrix with the given row vectors.
		 */
		explicit FORCE_INLINE Mat(const Vec<T, 1> vec[3])  : MatBase<T, 3, 1>(vec)
		{
		}

		/** This constructor initializes the matrix with the given row vectors.
		 */
		explicit FORCE_INLINE Mat(const Vec<T, 1> &v1, const Vec<T, 1> &v2, const Vec<T, 1> &v3)
		{
			v[0] = v1;
			v[1] = v2;
			v[2] = v3;
		}

		/** This constructor initializes the matrix with the given row vectors.
		 */
		explicit FORCE_INLINE Mat(const T &v1, const T &v2, const T &v3)
		{
			v[0][0] = v1;
			v[1][0] = v2;
			v[2][0] = v3;
		}

		/** Copy constructor 
		 */
		FORCE_INLINE Mat(const Mat<T, 3, 1>& m)  : MatBase<T, 3, 1>(m)
		{
		} 

		/** Conversion
		 */
		FORCE_INLINE Mat(const MatBase<T, 3, 1>& m)  : MatBase<T, 3, 1>(m)
		{
		} 

		FORCE_INLINE operator Vec<T, 3> ();

		using MatBase<T, 3, 1>::operator =;
	};

	// Matrix 1x3

	template<typename T>
	class Mat<T, 1, 3> : public MatBase<T, 1, 3>
	{
	public:
		/** Standard constructor
		 */
		FORCE_INLINE Mat() : MatBase<T, 1, 3>()
		{
		}

		/** This constructor can initialize the matrix.
		 */
		FORCE_INLINE Mat(bool initialize) : MatBase<T, 1, 3>(initialize)
		{
		}

		/** This constructor initializes the matrix with the given row vectors.
		 */
		explicit FORCE_INLINE Mat(const Vec<T, 3> vec[1])  : MatBase<T, 1, 3>(vec)
		{
		}

		/** This constructor initializes the matrix with the given row vectors.
		 */
		explicit FORCE_INLINE Mat(const Vec<T, 3> &v1)
		{
			v[0] = v1;
		}

		/** This constructor initializes the matrix with the given row vectors.
		 */
		explicit FORCE_INLINE Mat(const T &v1, const T &v2, const T &v3)
		{
			v[0][0] = v1;
			v[0][1] = v2;
			v[0][2] = v3;
		}

		/** Copy constructor 
		 */
		FORCE_INLINE Mat(const Mat<T, 1, 3>& m)  : MatBase<T, 1, 3>(m)
		{
		} 

		/** Conversion
		 */
		FORCE_INLINE Mat(const MatBase<T, 1, 3>& m)  : MatBase<T, 1, 3>(m)
		{
		} 

		FORCE_INLINE operator Vec<T, 3> ();

		using MatBase<T, 1, 3>::operator =;
	};


	// Matrix 4x4

	template<typename T>
	class Mat<T, 4, 4> : public MatNxN<T, 4>
	{
	public:
		/** Standard constructor
		 */
		FORCE_INLINE Mat() : MatNxN<T, 4>()
		{
		}

		/** This constructor can initialize the matrix.
		 */
		FORCE_INLINE Mat(bool initialize) : MatNxN<T, 4>(initialize)
		{
		}

		/** This constructor initializes the matrix with the given row vectors.
		 */
		explicit FORCE_INLINE Mat(const Vec<T, 4> vec[4])  : MatNxN<T, 4>(vec)
		{
		}

		/** This constructor initializes the matrix with the given row vectors.
		 */
		explicit FORCE_INLINE Mat(const Vec<T, 4> &v1, const Vec<T, 4> &v2, const Vec<T, 4> &v3, const Vec<T, 4> &v4)
		{
			v[0] = v1;
			v[1] = v2;
			v[2] = v3;
			v[3] = v4;
		}

		/** This constructor creates the matrix with the given values. 
		  */
		explicit FORCE_INLINE Mat(const T matrix[4][4])
		{ 
			for (int i=0; i < 4; i++)
				for (int j=0; j < 4; j++)
					v[i][j] = matrix[i][j];
		}

		/** This constructor creates the matrix with the given values. 
		  */
		explicit FORCE_INLINE Mat(const T matrix[16])
		{ 
			for (int i=0; i < 4; i++)
				for (int j=0; j < 4; j++)
					v[i][j] = matrix[i*4+j];
		}

		/** Copy constructor 
		 */
		FORCE_INLINE Mat(const Mat<T, 4, 4>& m)  : MatNxN<T, 4>(m)
		{
		} 

		/** Conversion
		 */
		FORCE_INLINE Mat(const MatNxN<T, 4>& m)  : MatNxN<T, 4>(m)
		{
		} 

		using MatBase<T, 4, 4>::operator =;

		/** Fills the matrix with the given values. 
		  */
		FORCE_INLINE void set(const T matrix[16])
		{ 
			for (int i=0; i < 4; i++)
				for (int j=0; j < 4; j++)
					v[i][j] = matrix[i*4+j];
		}

		/** Fills the matrix with the given values from a column-major array 
		  */
		FORCE_INLINE void setFromColumnMajor(const T matrix[16])
		{ 
			for (int i=0; i < 4; i++)
				for (int j=0; j < 4; j++)
					v[i][j] = matrix[i+j*4];
		}

		/** Get the matrix. 
		  */
		FORCE_INLINE void get(T *matrix)
		{ 
			for (int i=0; i < 4; i++)
				for (int j=0; j < 4; j++)
					matrix[i*4+j] = v[i][j];
		}

		/** Get the matrix in column major order 
		  */
		FORCE_INLINE void getColumnMajor(T *matrix)
		{ 
			for (int i=0; i < 4; i++)
				for (int j=0; j < 4; j++)				
					matrix[i+j*4] = v[i][j];
		}

		/** Fills the matrix with the given values. 
		  */
		FORCE_INLINE void setGL(const T matrix[16])
		{ 
			for (int i=0; i < 3; i++)
				for (int j=0; j < 3; j++)
					v[i][j] = matrix[i*4+j];

			v[0][3] = matrix[12];
			v[1][3] = matrix[13];
			v[2][3] = matrix[14];
			v[3][0] = (T) 0.0;
			v[3][1] = (T) 0.0;
			v[3][2] = (T) 0.0;
			v[3][3] = (T) 1.0;
		}

		/** Get the matrix. 
		  */
		FORCE_INLINE void getGL(T *matrix)
		{ 
			for (int i=0; i < 3; i++)
				for (int j=0; j < 3; j++)
					matrix[i*4+j] = v[i][j];

			matrix[12] = v[0][3];
			matrix[13] = v[1][3];
			matrix[14] = v[2][3];
			matrix[3] = (T) 0.0;
			matrix[7] = (T) 0.0;
			matrix[11] = (T) 0.0;
			matrix[15] = (T) 1.0;
		}

		/** Matrix product
		 */
		FORCE_INLINE static void mult(const Mat<T, 4, 4>& a, const Mat<T, 4, 4>& b, Mat<T, 4, 4> &c)
		{
			for (int i=0; i < 4; i++)
			{
				c[i][0] = a[i][0]*b[0][0] + a[i][1]*b[1][0] + a[i][2]*b[2][0] + a[i][3]*b[3][0];
				c[i][1] = a[i][0]*b[0][1] + a[i][1]*b[1][1] + a[i][2]*b[2][1] + a[i][3]*b[3][1];
				c[i][2] = a[i][0]*b[0][2] + a[i][1]*b[1][2] + a[i][2]*b[2][2] + a[i][3]*b[3][2];
				c[i][3] = a[i][0]*b[0][3] + a[i][1]*b[1][3] + a[i][2]*b[2][3] + a[i][3]*b[3][3];
			}
		}

		/** Matrix product\n
		  * This function is faster than the general matrix product
		  * but it only works for SE(3) transformation matrices. 
		  */
		FORCE_INLINE static void multTransform(const Mat<T, 4, 4>& a, const Mat<T, 4, 4>& b, Mat<T, 4, 4> &c) 
		{
			/* Rc = Ra Rb */
			c[0][0] = a[0][0]*b[0][0] + a[0][1]*b[1][0] + a[0][2]*b[2][0];
			c[0][1] = a[0][0]*b[0][1] + a[0][1]*b[1][1] + a[0][2]*b[2][1];
			c[0][2] = a[0][0]*b[0][2] + a[0][1]*b[1][2] + a[0][2]*b[2][2];
			c[1][0] = a[1][0]*b[0][0] + a[1][1]*b[1][0] + a[1][2]*b[2][0];
			c[1][1] = a[1][0]*b[0][1] + a[1][1]*b[1][1] + a[1][2]*b[2][1];
			c[1][2] = a[1][0]*b[0][2] + a[1][1]*b[1][2] + a[1][2]*b[2][2];
			c[2][0] = a[2][0]*b[0][0] + a[2][1]*b[1][0] + a[2][2]*b[2][0];
			c[2][1] = a[2][0]*b[0][1] + a[2][1]*b[1][1] + a[2][2]*b[2][1];
			c[2][2] = a[2][0]*b[0][2] + a[2][1]*b[1][2] + a[2][2]*b[2][2];

			/* Vc = Ra Vb + Va */
			c[0][3] = a[0][0]*b[0][3] + a[0][1]*b[1][3] + a[0][2]*b[2][3] + a[0][3];
			c[1][3] = a[1][0]*b[0][3] + a[1][1]*b[1][3] + a[1][2]*b[2][3] + a[1][3];
			c[2][3] = a[2][0]*b[0][3] + a[2][1]*b[1][3] + a[2][2]*b[2][3] + a[2][3];

			/* Rest */
			c[3][0] = c[3][1] = c[3][2] = 0.0;
			c[3][3] = 1.0;
		}

		/** Matrix vector product
		 */
		FORCE_INLINE static void multVector(const Mat<T, 4, 4>& m, const Vec<T, 3>& v, Vec<T, 3> &res)
		{
			res[0] = v[0]*m[0][0] + v[1]*m[0][1] + v[2]*m[0][2];
			res[1] = v[0]*m[1][0] + v[1]*m[1][1] + v[2]*m[1][2];
			res[2] = v[0]*m[2][0] + v[1]*m[2][1] + v[2]*m[2][2];
		}

		/** Matrix point product
		 */
		FORCE_INLINE static void multPoint(const Mat<T, 4, 4>& m, const Vec<T, 3>& p, Vec<T, 3> &res)
		{
			res[0] = p[0]*m[0][0] + p[1]*m[0][1] + p[2]*m[0][2] + m[0][3];
			res[1] = p[0]*m[1][0] + p[1]*m[1][1] + p[2]*m[1][2] + m[1][3];
			res[2] = p[0]*m[2][0] + p[1]*m[2][1] + p[2]*m[2][2] + m[2][3];
		}

		/** Scalar product
		 */
		FORCE_INLINE static void mult(const Mat<T, 4, 4>& m, const T& s, Mat<T, 4, 4> &res)
		{
			res[0] = s * m[0];
			res[1] = s * m[1];
			res[2] = s * m[2];
			res[3] = s * m[3];
		}


		/** Returns the inverse matrix. 
		  */
		Mat<T, 4, 4> inverseTransformationMatrix () const
		{
			Mat<T, 4, 4> inv;

			/* Invertieren des Rotationsteils durch Transponieren */
			inv[0][0] = v[0][0];
			inv[0][1] = v[1][0];
			inv[0][2] = v[2][0];
			inv[1][0] = v[0][1];
			inv[1][1] = v[1][1];
			inv[1][2] = v[2][1];
			inv[2][0] = v[0][2];
			inv[2][1] = v[1][2];
			inv[2][2] = v[2][2];

			/* Verschiebungsvektor:  d' = -(R^-1) * d */
			inv[0][3] = - inv[0][0]*v[0][3] - inv[0][1]*v[1][3] - inv[0][2]*v[2][3];
			inv[1][3] = - inv[1][0]*v[0][3] - inv[1][1]*v[1][3] - inv[1][2]*v[2][3];
			inv[2][3] = - inv[2][0]*v[0][3] - inv[2][1]*v[1][3] - inv[2][2]*v[2][3];

			/* Der Rest bleibt gleich */
			inv[3][0] = inv[3][1] = inv[3][2] = 0.0;
			inv[3][3] = 1.0;

			return inv;
		}

		/** Return the translation, rotation and scale of this tranformation matrix
		  */
		void getTransformation (Vec<T, 3> &translation, Mat<T, 3, 3> &rotation, Vec<T, 3> &scale) const
		{
			// compute rotation matrix
			rotation = Mat<T, 3, 3> (	Vec<T, 3> (v[0][0], v[0][1], v[0][2]), 
											Vec<T, 3> (v[1][0], v[1][1], v[1][2]),
											Vec<T, 3> (v[2][0], v[2][1], v[2][2]));
			scale[0] = rotation[0].length ();
			scale[1] = rotation[1].length ();
			scale[2] = rotation[2].length ();

			// Remove scale of rotation matrix
			rotation[0] = 1.0/scale[0] * rotation[0];
			rotation[1] = 1.0/scale[1] * rotation[1];
			rotation[2] = 1.0/scale[2] * rotation[2];

			translation = Vec<T, 3> (v[0][3], v[1][3], v[2][3]);
		}

		/** Sets the translation, rotation and scale of this transformation matrix
		  */
		void setTransformation (Vec<T, 3> &translation, Mat<T, 3, 3> &rotation, Vec<T, 3> &scale)
		{
			v[0][0] = scale[0] * rotation[0][0];
			v[0][1] = rotation[0][1];
			v[0][2] = rotation[0][2];

			v[1][0] = rotation[1][0];
			v[1][1] = scale[1] * rotation[1][1];
			v[1][2] = rotation[1][2];

			v[2][0] = rotation[2][0];
			v[2][1] = rotation[2][1];
			v[2][2] = scale[2] * rotation[2][2];			

			v[0][3] = translation[0];
			v[1][3] = translation[1];
			v[2][3] = translation[2];
			v[3][3] = (T) 1.0;

			v[3][0] = (T) 0.0;
			v[3][1] = (T) 0.0;
			v[3][2] = (T) 0.0;
		}
	};

	template<typename T>
	FORCE_INLINE const Mat<T, 4, 4> operator * (const Mat<T, 4, 4>& a, const Mat<T, 4, 4>& b)
	{
		Mat<T, 4, 4> res;
		Mat<T, 4, 4>::mult(a, b, res);
		return res;
	}

	template<typename T>
	FORCE_INLINE const Mat<T, 4, 4> operator ^ (const Mat<T, 4, 4>& a, const Mat<T, 4, 4>& b)
	{
		Mat<T, 4, 4> res;
		Mat<T, 4, 4>::multTransorm(a, b, res);
		return res;
	}

	template<typename T>
	FORCE_INLINE const Vec<T, 3> operator ^ (const Mat<T, 4, 4>& m, const Vec<T, 3>& v)
	{
		Vec<T, 3> res;
		Mat<T, 4, 4>::multPoint(m, v, res);
		return res;
	}

	template<typename T>
	FORCE_INLINE const Vec<T, 3> operator * (const Mat<T, 4, 4>& m, const Vec<T, 3>& v)
	{
		Vec<T, 3> res;
		Mat<T, 4, 4>::multVector(m, v, res);
		return res;
	}

	typedef Mat<Real, 3, 3>   Matrix3x3;
	typedef Mat<Real, 2, 3>   Matrix2x3;
	typedef Mat<Real, 3, 2>   Matrix3x2;
	typedef Mat<Real, 2, 2>   Matrix2x2;
	typedef Mat<Real, 4, 4>   Matrix4x4;
	typedef Mat<Real, 6, 6>   Matrix6x6;
	typedef Mat<Real, 6, 12>  Matrix6x12;
	typedef Mat<Real, 12, 6>  Matrix12x6;
	typedef Mat<Real, 12, 12> Matrix12x12;

}

#endif


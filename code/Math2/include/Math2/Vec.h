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

#ifndef __VEC_H__
#define __VEC_H__

#include "Common/Config.h"
#include "MathDefs.h"
#include "VecBase.h"
#include <iostream>
//#include <emmintrin.h>


namespace IBDS 
{
	/** Vec is a template class for vector operations.
	 */
	template<typename T, int size>
	class Vec : public VecBase<T, size>
	{
	public:
		/** Standard constructor
		 */
		FORCE_INLINE Vec() : VecBase<T, size>()
		{
		}
		
		/** This constructor can initialize the matrix.
		 */
		FORCE_INLINE Vec(bool initialize) : VecBase<T, size>(initialize)
		{
		}

		/** Copy constructor 
		 */
		FORCE_INLINE Vec(const Vec& v)  : VecBase<T, size>(v)
		{
		}

		/** Conversion
		 */
		FORCE_INLINE Vec(const VecBase<T, size>& v)  : VecBase<T, size>(v)
		{
		}

		using VecBase<T, size>::operator =;
		using VecBase<T, size>::operator ();
		using VecBase<T, size>::operator [];
	};



	template<typename T, int size>
	FORCE_INLINE const Vec<T, size> operator +(const Vec<T, size>& v1, const Vec<T, size>& v2)
	{
		Vec<T, size> res;
		Vec<T, size>::add(v1, v2, res);
		return res;
	}

	template<typename T, int size>
	FORCE_INLINE const Vec<T, size> operator -(const Vec<T, size>& v1, const Vec<T, size>& v2)
	{
		Vec<T, size> res;
		Vec<T, size>::sub(v1, v2, res);
		return res;
	}

	template<typename T, int size>
	FORCE_INLINE const Vec<T, size> operator *(const Vec<T, size>& v, const T& s)
	{
		Vec<T, size> res;
		Vec<T, size>::mult(v, s, res);
		return res;
	}

	template<typename T, int size>
	FORCE_INLINE const Vec<T, size> operator *(const T& s, const Vec<T, size>& v)
	{
		Vec<T, size> res;
		Vec<T, size>::mult(s, v, res);
		return res;
	}

	// Negation
	template<typename T, int size>
	FORCE_INLINE const Vec<T, size> operator - (const Vec<T, size>& v)
	{
		Vec<T, size> res;
		Vec<T, size>::neg(v, res);
		return res;
	}

	// Dot product
	template<typename T, int size>
	FORCE_INLINE const T operator *(const Vec<T, size>& v1, const Vec<T, size>& v2)
	{
		T res;
		Vec<T, size>::mult(v1, v2, res);
		return res;
	}

	// Multiplication of two diagonal matrices (stored in a vector)
	template<typename T, int size>
	FORCE_INLINE const Vec<T, size> operator |(const Vec<T, size>& v1, const Vec<T, size>& v2)
	{
		Vec<T, size> res;
		Vec<T, size>::mult(v1, v2, res);
		return res;
	}

	// Equal
	template<typename T, int size>
	FORCE_INLINE const bool operator ==(const Vec<T, size>& v1, const Vec<T, size>& v2)
	{
		bool res;
		Vec<T, size>::equal(v1, v2, res);
		return res;
	}

	// Not equal
	template<typename T, int size>
	FORCE_INLINE const bool operator !=(const Vec<T, size>& v1, const Vec<T, size>& v2)
	{
		bool res;
		Vec<T, size>::unequal(v1, v2, res);
		return res;
	}

	/** Less than
     */
	template<typename T, int size>
	FORCE_INLINE const bool operator < (const Vec<T, size>& v1, const Vec<T, size>& v2)
	{
		bool res;
		Vec<T, size>::less(v1, v2, res);
		return res;
	}

	// Vec4

	template<typename T>
	class Vec<T, 4> : public VecBase<T, 4>
	{
	public:
		/** Standard constructor
		 */
		FORCE_INLINE Vec() : VecBase<T, 4>()
		{
		}
		
		/** This constructor can initialize the matrix.
		 */
		FORCE_INLINE Vec(bool initialize) : VecBase<T, 4>(initialize)
		{
		}

		/** Copy constructor 
		 */
		FORCE_INLINE Vec(const Vec& v)  : VecBase<T, 4>(v)
		{
		}

		/** Conversion
		 */
		FORCE_INLINE Vec(const VecBase<T, 4>& v)  : VecBase<T, 4>(v)
		{
		}		

		FORCE_INLINE Vec(const T &a, const T &b, const T &c, const T &d) 
		{
			this->elem[0] = a;
			this->elem[1] = b;
			this->elem[2] = c;
			this->elem[3] = d;
		} 

		using VecBase<T, 4>::operator =;
		using VecBase<T, 4>::operator ();
		using VecBase<T, 4>::operator [];
	};

	template<typename T, int rows, int cols>
	class Mat;


	// Vec3

	template<typename T>
	class Vec<T, 3> : public VecBase<T, 3>
	{
	public:
		/** Standard constructor
		 */
		FORCE_INLINE Vec() : VecBase<T, 3>()
		{
		}
		
		/** This constructor can initialize the matrix.
		 */
		FORCE_INLINE Vec(bool initialize) : VecBase<T, 3>(initialize)
		{
		}

		/** Copy constructor 
		 */
		FORCE_INLINE Vec(const Vec& v)  : VecBase<T, 3>(v)
		{
		}

		/** Conversion
		 */
		FORCE_INLINE Vec(const VecBase<T, 3>& v)  : VecBase<T, 3>(v)
		{
		}		

		FORCE_INLINE Vec(const T &a, const T &b, const T &c) 
		{
			this->elem[0] = a;
			this->elem[1] = b;
			this->elem[2] = c;
		} 

		/** Cross product
		 */
		FORCE_INLINE static void cross(const Vec<T, 3>& a, const Vec<T, 3>& b, Vec<T, 3> &res)
		{
			res[0] = a[1]*b[2] - a[2]*b[1];
			res[1] = a[2]*b[0] - a[0]*b[2];
			res[2] = a[0]*b[1] - a[1]*b[0];
		}

		/** Compute the angle between the two vectors
		 */
		FORCE_INLINE static T angle(const Vec<T, 3>& a, const Vec<T, 3>& b)
		{
			const T al = a.length();
			const T bl = b.length();
			const T div = al*bl;
			if (fabs(div) > EPSILON)
			{
				T c = a*b/div;
				if (c > 1.0) 
					c = 1.0;
				else if (c < -1.0) 
					c = -1.0;
				return fastACos (c);
			}
			else 
				return 0.0;
		}

		// Geometric Tools, LLC
		// Copyright (c) 1998-2010
		// Distributed under the Boost Software License, Version 1.0.
		// http://www.boost.org/LICENSE_1_0.txt
		// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
		FORCE_INLINE static T fastACos (const Real value)
		{
			T root = sqrt(fabs((Real)1 - value));
			T result = -(T)0.0012624911;
			result *= value;
			result += (T)0.0066700901;
			result *= value;
			result -= (T)0.0170881256;
			result *= value;
			result += (T)0.0308918810;
			result *= value;
			result -= (T)0.0501743046;
			result *= value;
			result += (T)0.0889789874;
			result *= value;
			result -= (T)0.2145988016;
			result *= value;
			result += (T)1.5707963050;
			result *= root;
			return result;
		}

		/** Compute the angle between the two normalized vectors. 
		 */
		FORCE_INLINE static T angleN(const Vec<T, 3>& a, const Vec<T, 3>& b)
		{
			T c = a*b;
			if (c > 1.0) 
				c = 1.0;
			else if (c < -1.0) 
				c = -1.0;
			return fastACos (c);
		}

		FORCE_INLINE operator Mat<T, 3, 1> ();
		FORCE_INLINE operator Mat<T, 1, 3> ();

		using VecBase<T, 3>::operator =;
		using VecBase<T, 3>::operator ();
		using VecBase<T, 3>::operator [];
	};


	/** Cross product
	 */
	template<typename T>
	FORCE_INLINE const Vec<T, 3> operator ^(const Vec<T, 3>& v1, const Vec<T, 3>& v2)
	{
		Vec<T, 3> res;
		Vec<T, 3>::cross(v1, v2, res);
		return res;
	}



	// Vec2

	template<typename T>
	class Vec<T, 2> : public VecBase<T, 2>
	{
	public:
		/** Standard constructor
		 */
		FORCE_INLINE Vec() : VecBase<T, 2>()
		{
		}
		
		/** This constructor can initialize the matrix.
		 */
		FORCE_INLINE Vec(bool initialize) : VecBase<T, 2>(initialize)
		{
		}

		/** Copy constructor 
		 */
		FORCE_INLINE Vec(const Vec& v)  : VecBase<T, 2>(v)
		{
		}

		/** Conversion
		 */
		FORCE_INLINE Vec(const VecBase<T, 2>& v)  : VecBase<T, 2>(v)
		{
		}		

		FORCE_INLINE Vec(const T &a, const T &b) 
		{
			this->elem[0] = a;
			this->elem[1] = b;
		} 

		using VecBase<T, 2>::operator =;
		using VecBase<T, 2>::operator ();
		using VecBase<T, 2>::operator [];
	};

	// Vec1

	template<typename T>
	class Vec<T, 1> : public VecBase<T, 1>
	{
	public:
		/** Standard constructor
		 */
		FORCE_INLINE Vec() : VecBase<T, 1>()
		{
		}
		
		/** This constructor can initialize the matrix.
		 */
		FORCE_INLINE Vec(bool initialize) : VecBase<T, 1>(initialize)
		{
		}

		/** Copy constructor 
		 */
		FORCE_INLINE Vec(const Vec& v)  : VecBase<T, 1>(v)
		{
		}

		/** Conversion
		 */
		FORCE_INLINE Vec(const VecBase<T, 1>& v)  : VecBase<T, 1>(v)
		{
		}		

		FORCE_INLINE Vec(const T &a) 
		{
			this->elem[0] = a;
		} 

		using VecBase<T, 1>::operator =;
		using VecBase<T, 1>::operator ();
		using VecBase<T, 1>::operator [];
	};

	//Vec6

	template<typename T>
	class Vec<T, 6> : public VecBase<T, 6>
	{
	public:
		/** Standard constructor
		 */
		FORCE_INLINE Vec() : VecBase<T, 6>()
		{
		}
		
		/** This constructor can initialize the matrix.
		 */
		FORCE_INLINE Vec(bool initialize) : VecBase<T, 6>(initialize)
		{
		}

		/** Copy constructor 
		 */
		FORCE_INLINE Vec(const Vec& v)  : VecBase<T, 6>(v)
		{
		}

		/** Conversion
		 */
		FORCE_INLINE Vec(const VecBase<T, 6>& v)  : VecBase<T, 6>(v)
		{
		}		

		FORCE_INLINE Vec(const T &a, const T &b, const T &c, const T &d, const T &e, const T &f) 
		{
			this->elem[0] = a;
			this->elem[1] = b;
			this->elem[2] = c;
			this->elem[3] = d;
			this->elem[4] = e;
			this->elem[5] = f;
		} 

		FORCE_INLINE Vec(const VecBase<T, 3> &first, const VecBase<T, 3> &second) 
		{
			this->elem[0] = first[0];
			this->elem[1] = first[1];
			this->elem[2] = first[2];
			this->elem[3] = second[0];
			this->elem[4] = second[1];
			this->elem[5] = second[2];
		} 

		using VecBase<T, 6>::operator =;
		using VecBase<T, 6>::operator ();
		using VecBase<T, 6>::operator [];
	};

	typedef Vec<Real, 1> Vector1D;
	typedef Vec<Real, 2> Vector2D;
	typedef Vec<Real, 3> Vector3D;
	typedef Vec<Real, 6> Vector6D;
	typedef Vec<Real, 12> Vector12D;



	//// Vec3

	//template<>
	//class Vec<float, 3> : public VecBase<float, 3>
	//{
	//public:
	//	/** Standard constructor
	//	 */
	//	FORCE_INLINE Vec() : VecBase<float, 3>()
	//	{
	//	}
	//	
	//	/** This constructor can initialize the matrix.
	//	 */
	//	FORCE_INLINE Vec(bool initialize) : VecBase<float, 3>(initialize)
	//	{
	//	}

	//	/** Copy constructor 
	//	 */
	//	FORCE_INLINE Vec(const Vec& v)  : VecBase<float, 3>(v)
	//	{
	//	}

	//	/** Conversion
	//	 */
	//	FORCE_INLINE Vec(const VecBase& v)  : VecBase<float, 3>(v)
	//	{
	//	}		

	//	FORCE_INLINE Vec(const float &a, const float &b, const float &c) 
	//	{
	//		this->elem[0] = a;
	//		this->elem[1] = b;
	//		this->elem[2] = c;
	//	} 

	//	/** Cross product
	//	 */
	//	FORCE_INLINE static void cross(const Vec<float, 3>& a, const Vec<float, 3>& b, Vec<float, 3> &res)
	//	{
	//		res[0] = a[1]*b[2] - a[2]*b[1];
	//		res[1] = a[2]*b[0] - a[0]*b[2];
	//		res[2] = a[0]*b[1] - a[1]*b[0];
	//	}

	//	using VecBase<float, 3>::operator =;
	//	using VecBase<float, 3>::operator ();
	//	using VecBase<float, 3>::operator [];
	//};

	///** Addition
 //    */
	//FORCE_INLINE const Vec<float, 3> operator +(const Vec<float, 3> &v1,const Vec<float, 3> &v2)
	//{
	//	__m128 t0, t1; 
	//	Vec<float, 3> res;
	//	t0 = _mm_load_ps(v1.elem); 
	//	t1 = _mm_load_ps(v2.elem);   
	//	t0 = _mm_add_ps(t0, t1); 	
	//	_mm_store_ps(res.elem, t0); 
	//	return res;
	//}


	//// Vec3

	//template<>
	//class Vec<double, 3> : public VecBase<double, 3>
	//{
	//public:
	//	/** Standard constructor
	//	 */
	//	FORCE_INLINE Vec() : VecBase<double, 3>()
	//	{
	//	}
	//	
	//	/** This constructor can initialize the matrix.
	//	 */
	//	FORCE_INLINE Vec(bool initialize) : VecBase<double, 3>(initialize)
	//	{
	//	}

	//	/** Copy constructor 
	//	 */
	//	FORCE_INLINE Vec(const Vec& v)  : VecBase<double, 3>(v)
	//	{
	//	}

	//	/** Conversion
	//	 */
	//	FORCE_INLINE Vec(const VecBase& v)  : VecBase<double, 3>(v)
	//	{
	//	}		

	//	FORCE_INLINE Vec(const double &a, const double &b, const double &c) 
	//	{
	//		this->elem[0] = a;
	//		this->elem[1] = b;
	//		this->elem[2] = c;
	//	} 

	//	/** Cross product
	//	 */
	//	FORCE_INLINE static void cross(const Vec<double, 3>& a, const Vec<double, 3>& b, Vec<double, 3> &res)
	//	{
	//		res[0] = a[1]*b[2] - a[2]*b[1];
	//		res[1] = a[2]*b[0] - a[0]*b[2];
	//		res[2] = a[0]*b[1] - a[1]*b[0];
	//	}

	//	using VecBase<double, 3>::operator =;
	//	using VecBase<double, 3>::operator ();
	//	using VecBase<double, 3>::operator [];
	//};

	///** Addition
 //    */
	//FORCE_INLINE Vec<double, 3> operator +(const Vec<double, 3> &v1,const Vec<double, 3> &v2)
	//{
	//	__m128d t0, t1, t2, t3; 
	//	Vec<double, 3> res;
	//	t0 = _mm_load_pd(&v1.this->elem[0]); 
	//	t1 = _mm_load_pd(&v2.this->elem[0]);   
	//	t2 = _mm_load_sd(&v1.this->elem[2]); 
	//	t3 = _mm_load_sd(&v2.this->elem[2]);   
	//	t0 = _mm_add_pd(t0, t1); 	
	//	t2 = _mm_add_sd(t2, t3); 	
	//	_mm_store_pd(&res.this->elem[0], t0); 
	//	_mm_store_sd(&res.this->elem[2], t2); 
	//	return res;
	//}
}

#endif


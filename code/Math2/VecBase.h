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

#ifndef __VECBASE_H__
#define __VECBASE_H__

#include "Common/Config.h"
#include "MatBase.h"
#include "MathDefs.h"
#include <iostream>
#include <math.h>

namespace IBDS 
{
	// forward declaration
	template<typename T, int size>
	class VecBase;

	class Vector : public Matrix
	{
	};

	template<typename T, int size> std::ostream& operator <<(std::ostream&, const VecBase<T, size>&);


	/** VecBase is a base template class for vector operations.
	 */
	template<typename T, int size>
	class VecBase : public Vector
	{
	protected:
		//__declspec(align(16)) T elem[size];
		T elem[size];

	public:
		/** Standard constructor
		 */
		FORCE_INLINE VecBase()
		{
		}
		
		/** This constructor can initialize the vector.
		 */
		FORCE_INLINE VecBase(bool initialize)
		{
			if (initialize)
			{
				zero();
			}
		}

		/** Copy constructor 
		 */
		FORCE_INLINE VecBase(const VecBase& v)
		{
			for(int i=0; i < size; i++)
			{
				elem[i] = v.elem[i];
			}
		}

		
		/** Sets all elements to zero 
		 */
		FORCE_INLINE void zero()
		{
			for(int i=0; i < size; i++)
			{
				elem[i] = 0;
			}
		}

		/** Normalize this vector.
		 */
		FORCE_INLINE void normalize() 
		{
			*this /= length(); 
		}

		/** Return the distance of this point to p.
		 */
		FORCE_INLINE T distance(const VecBase<T, size>& v) const 
		{
			VecBase<T, size> diff;
			sub(*this, v, diff);
			return diff.length();
		}

		/** Return the squared distance of this point to p.
		 */
		FORCE_INLINE T distance2(const VecBase<T, size>& v) const 
		{
			VecBase<T, size> diff;
			sub(*this, v, diff);
			return diff.length2 ();
		}

		/** Return the magnitude of the vector.
		 */
		FORCE_INLINE T length() const 
		{
			T dot;
			mult(*this, *this, dot);
			return (T) sqrt (dot); 
		}

		/** Return the square magnitude of the vector.
		 */
		FORCE_INLINE T length2() const 
		{
			T dot;
			mult(*this, *this, dot);
			return dot; 
		}

		/** Return the number of columns.
		 */
		FORCE_INLINE int getCols() const
		{
			return 1;
		}

		/** Return the number of rows.
		 */
		FORCE_INLINE int getRows() const
		{
			return size;
		}

		FORCE_INLINE T* getElements()
		{
			return &elem[0];
		}

		// member operators
		FORCE_INLINE VecBase& operator =(const VecBase& v)
		{
			memcpy (elem, v.elem, sizeof(T)*size);
			return *this; 
		}


		FORCE_INLINE T& operator [] (const int i)
		{
			return elem[i];
		}

		FORCE_INLINE const T& operator [] (const int i) const
		{
			return elem[i];
		}

		FORCE_INLINE Real& operator () (int i, int j)
		{
			return elem[i];
		}

		FORCE_INLINE const Real& operator () (int i, int j) const
		{
			return elem[i];
		}

		FORCE_INLINE VecBase& operator +=(const VecBase& v)
		{
			for(int i=0; i < size; i++)
			{
				elem[i] += v.elem[i];
			}
			return *this;
		}

		FORCE_INLINE VecBase& operator -=(const VecBase& v)
		{
			for(int i=0; i < size; i++)
			{
				elem[i] -= v.elem[i];
			}
			return *this;
		}

		FORCE_INLINE VecBase& operator *=(const T& s)
		{
			for(int i=0; i < size; i++)
			{
				elem[i] *= s;
			}
			return *this;
		}

		FORCE_INLINE VecBase& operator /=(const T& s)
		{
			for(int i=0; i < size; i++)
			{
				elem[i] /= s;
			}
			return *this;
		}

		friend std::ostream& operator<< <T, size>(std::ostream&, const VecBase<T, size>&);

		/** Scalar product
		 */
		FORCE_INLINE static void mult(const VecBase<T, size>& v, const T& s,  VecBase<T, size> &res)
		{
			for (int i=0; i < size; i++)
			{
				res[i] = s * v[i];
			}
		}

		/** Scalar product
		 */
		FORCE_INLINE static void mult(const T& s,  const VecBase<T, size>& v, VecBase<T, size> &res)
		{
			for (int i=0; i < size; i++)
			{
				res[i] = s * v[i];
			}
		}

		/** Dot product
		 */
		FORCE_INLINE static void mult(const VecBase<T, size>& v1, const VecBase<T, size>& v2, T &res)
		{
			res = (T) 0;
			for (int i=0; i < size; i++)
			{
				res += v1[i]*v2[i];
			}
		}

		/** Multiplication of two diagonal matrices (stored in a vector)
		 */
		FORCE_INLINE static void mult(const VecBase<T, size>& v1, const VecBase<T, size>& v2, VecBase<T, size> &res)
		{
			for (int i=0; i < size; i++)
			{
				res[i] = v1[i]*v2[i];
			}
		}

		/** Multiplication of v1 * v2^T
		 */
		FORCE_INLINE static void mult(const VecBase<T, size>& v1, const VecBase<T, size>& v2, MatBase<T, size, size> &res)
		{
			for (int i=0; i < size; i++)
			{
				for (int j=0; j < size; j++)
					res(i,j) = v1[i]*v2[j];
			}
		}

		/** Substraction
		 */
		FORCE_INLINE static void sub(const VecBase<T, size>& v1, const VecBase<T, size>& v2, VecBase<T, size> &res)
		{
			for (int i=0; i < size; i++)
			{
				res[i] = v1[i] - v2[i];
			}
		}

		/** Addition
		 */
		FORCE_INLINE static void add(const VecBase<T, size>& v1, const VecBase<T, size>& v2, VecBase<T, size> &res)
		{
			for (int i=0; i < size; i++)
			{
				res[i] = v1[i] + v2[i];
			}
		}

		/** Negation 
		 */
		FORCE_INLINE static void neg(const VecBase<T, size> &v, VecBase<T, size> &res)
		{
			for (int i=0; i < size; i++)
			{
				res[i] = -v[i];
			}
		}

		/** Equality
		 */
		FORCE_INLINE static void equal(const VecBase<T, size>& v1, const VecBase<T, size>& v2, bool &res)
		{
			res = true;
			for (int i=0; (i < size) && res; i++)
			{
				res = res && (v1[i] == v2[i]);
			}
		}

		/** Inequality
		 */
		FORCE_INLINE static void unequal(const VecBase<T, size>& v1, const VecBase<T, size>& v2, bool &res)
		{
			res = false;
			for (int i=0; (i < size) && (!res); i++)
			{
				res = res || (v1[i] != v2[i]);
			}
		}

		/** Less than
		 */
		FORCE_INLINE static void less(const VecBase<T, size>& a, const VecBase<T, size>& b, bool &res)
		{
			res = false;
			for (int i=0; i < size; i++)
			{
				if (a[i] < b[i])
				{
					res = true;
					return;
				}
				else if (a[i] > b[i])
				{
					res = false;
					return;
				}
			}
		}
	};



	template<typename T, int size>
	std::ostream& operator <<(std::ostream& s, const VecBase<T, size>& v)
	{
		s << "( ";
		for (int i=0; i < size; i++)
		{
			s << v.elem[i] << ", ";
		}
		s << " )"; 
		return s;
	}


}

#endif


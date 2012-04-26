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

#ifndef __MATBASE_H__
#define __MATBASE_H__

#include "Common/Config.h"
#include <iostream>

namespace IBDS
{
	template<typename T, int size>
	class Vec;

	class Matrix
	{
	};

	// forward declaration
	template<typename T, int rows, int cols>
	class MatBase;

	template<typename T, int rows, int cols> std::ostream& operator <<(std::ostream&, const MatBase<T, rows, cols>&);

	/** MatBase is a template class for matrix operations.
	 */
	template<typename T, int rows, int cols>
	class MatBase : public Matrix
	{
	protected:
		/** Row vectors of the matrix */
		Vec<T, cols> v[rows];

	public:
		/** Standard constructor
		 */
		FORCE_INLINE MatBase()
		{
		}

		/** This constructor can initialize the matrix.
		 */
		FORCE_INLINE MatBase(bool initialize)
		{
			if (initialize)
			{
				zero();
			}
		}

		/** This constructor initializes the matrix with the given row vectors.
		 */
		explicit FORCE_INLINE MatBase(const Vec<T, cols> vec[rows])
		{
			for (int i=0; i < rows; i++)
			{
				v[i] = vec[i];
			}
		}

		/** Copy constructor 
		 */
		FORCE_INLINE MatBase(const MatBase& m)
		{
			for(int i=0; i < rows; i++)
			{
				v[i] = m.v[i];
			}
		} 
		
		/** Sets all elements to zero 
		 */
		FORCE_INLINE void zero()
		{
			for(int i=0; i < rows; i++)
			{
				for(int j=0; j < cols; j++)
				{
					v[i][j] = 0;
				}
			}
		}

		FORCE_INLINE Real* getElements()
		{
			return reinterpret_cast<Real*>(v);
		}

		FORCE_INLINE int getRows () const
		{
			return rows;
		}

		FORCE_INLINE int getCols () const
		{
			return cols;
		}		

		/** Transpose 
		 */
		FORCE_INLINE MatBase<T, cols, rows> transpose () const
		{
			MatBase<T, cols, rows> res;
			MatBase<T, rows, cols>::transpose(*this, res);
			return res;
		}

		/** Returns the inverse matrix. 
		  */
		bool inverse () 
		{
			int *colIndex = new int [rows];
			int *rowIndex = new int [rows];;
			bool *pivot = new bool [rows];;

			memset (pivot, 0, rows * sizeof(bool));

			for (int i = 0; i < rows; i++) 
			{
				double maxVal = 0.0;
				int row = 0;
				int col = 0;
				bool chk = false;
				for (int j = 0; j < rows; j++) 
				{
					if (!pivot[j]) 
					{
						for (int k = 0; k < rows; k++) 
						{
							if (!pivot[k]) 
							{
								double val = fabs (v[j][k]);
								if (val > maxVal) 
								{
									maxVal = val;
									row = j;
									col = k;
									chk = true;
								}
							}
						}
					}
				}
				if (!chk) 
					return false;

				pivot[col] = true;

				if (row != col) 
				{
					double *ptr;
					ptr = new double [cols];
					memcpy (ptr, &v[row][0], cols * sizeof (double));
					memcpy (&v[row][0], &v[col][0], cols * sizeof (double));
					memcpy (&v[col][0], ptr, cols * sizeof(double));
					delete [] ptr;
				}

				rowIndex[i] = row;
				colIndex[i] = col;

				double scale = 1.0 / v[col][col];
				v[col][col] = 1.0;
				for (int k = 0; k < rows; k++)
					v[col][k] *= scale;

				for (int j = 0; j < rows; j++) 
				{
					if (j != col) 
					{
						scale = v[j][col];
						v[j][col] = 0.0;
						for (int k = 0; k < rows; k++ )
							v[j][k] -= v[col][k] * scale;
					}
				}
			}

			for (int j = rows - 1; j >= 0; j--) 
			{
				if (rowIndex[j] != colIndex[j]) 
				{
					for (int k = 0; k < rows; k++) 
					{
						double val = v[k][rowIndex[j]];
						v[k][rowIndex[j]] = v[k][colIndex[j]];
						v[k][colIndex[j]] = val;
					}
				}
			}

			delete [] colIndex;
			delete [] rowIndex;
			delete [] pivot;

			return true;
		}

		/** Matrix product.
		 * Be careful: res must not be a reference to m1 or m2
		 */
		template<typename T, int rows, int cols, int cols2>
		FORCE_INLINE static void mult(const MatBase<T, rows, cols>& m1, const MatBase<T, cols, cols2>& m2, MatBase<T, rows, cols2> &res)
		{
			for (int i=0; i < rows; i++)
			{
				for (int j=0; j < cols2; j++)
				{
					res(i, j) = 0;
					for (int k=0; k < cols; k++)
					{
						res(i, j) += m1(i, k) * m2(k, j);
					}
				}
			}
		}

		/** Matrix product: m1^T * m2
		 * Be careful: res must not be a reference to m1 or m2
		 */
		template<typename T, int rows, int cols, int cols2>
		FORCE_INLINE static void multFirstTransposed(const MatBase<T, cols, rows>& m1, const MatBase<T, cols, cols2>& m2, MatBase<T, rows, cols2> &res)
		{
			for (int i=0; i < rows; i++)
			{
				for (int j=0; j < cols2; j++)
				{
					res(i, j) = 0;
					for (int k=0; k < cols; k++)
					{
						res(i, j) += m1(k, i) * m2(k, j);
					}
				}
			}
		}

		/** Matrix vector product
		 */
		FORCE_INLINE static void mult(const MatBase<T, rows, cols>& m, const Vec<T, cols>& v, Vec<T, rows> &res)
		{
			for (int i=0; i < rows; i++)
			{
				res[i] = 0;
				for (int j=0; j < cols; j++)
				{
					res[i] += m(i, j) * v[j];
				}
			}
		}

		/** Vector matrix product
		 */
		FORCE_INLINE static void mult(const Vec<T, rows>& v, const MatBase<T, rows, cols>& m,  Vec<T, cols> &res)
		{
			for (int i=0; i < cols; i++)
			{
				res[i] = 0;
				for (int j=0; j < rows; j++)
				{
					res[i] += v[j] * m(j, i);
				}
			}
		}

		/** Scalar product
		 */
		FORCE_INLINE static void mult(const MatBase<T, rows, cols>& m, const Real& s, MatBase<T, rows, cols> &res)
		{
			for (int i=0; i < rows; i++)
			{
				res[i] = s * m[i];
			}
		}

		/** Negation 
		 */
		FORCE_INLINE static void neg(const MatBase<T, rows, cols>& m, MatBase<T, rows, cols> &res)
		{
			for (int i=0; i < rows; i++)
			{
				res[i] = -m[i];
			}
		}

		/** Substraction
		 */
		FORCE_INLINE static void sub(const MatBase<T, rows, cols>& m1, const MatBase<T, rows, cols>& m2, MatBase<T, rows, cols> & res)
		{
			for(int i=0; i < rows; i++)
			{
				res[i] = m1[i] - m2[i];
			}
		}

		/** Addition
		 */
		FORCE_INLINE static void add(const MatBase<T, rows, cols>& m1, const MatBase<T, rows, cols>& m2, MatBase<T, rows, cols> & res)
		{
			for(int i=0; i < rows; i++)
			{
				res[i] = m1[i] + m2[i];
			}
		}

		/** Transpose 
		 */
		FORCE_INLINE static void transpose(const MatBase<T, rows, cols>& m, MatBase<T, cols, rows> &res)
		{
			for (int i=0; i < rows; i++)
			{
				for (int j=0; j < cols; j++)
				{
					res[j][i] = m[i][j];
				}
			}
		}

		// member operators
		FORCE_INLINE MatBase& operator =(const MatBase& m)
		{
			for (int i=0; i < rows; i++)
			{
				v[i] = m.v[i];
			}
			return *this; 
		}

		FORCE_INLINE MatBase& operator +=(const MatBase& m)
		{
			add(*this, m, *this);
			return *this;
		}

		FORCE_INLINE MatBase& operator -=(const MatBase& m)
		{
			sub(*this, m, *this);
			return *this;
		}

		FORCE_INLINE MatBase& operator *=(const Real s)
		{
			mult(*this, s, *this);
			return *this;
		}

		FORCE_INLINE Vec<T, cols>& operator [] (const int i)
		{
			return v[i];
		}

		FORCE_INLINE const Vec<T, cols>& operator [] (const int i) const
		{
			return v[i];
		}

        FORCE_INLINE Real& operator () (int i, int j)
		{
			return v[i][j];
		}

        FORCE_INLINE const Real& operator () (int i, int j) const
		{
			return v[i][j];
		}		

		/** set subblock of matrix to values of other matrix
		 */
		template<typename T, int rowsB, int colsB, int rowsT, int colsT>
		FORCE_INLINE static void setBlock(unsigned int topLeftRow, unsigned int topLeftColumn, const MatBase<T, rowsB, colsB>& block, MatBase<T, rowsT, colsT> &target)
		{
			int columnCnt = std::min<int>(colsT - topLeftColumn , colsB);
			int rowCnt = std::min<int>(rowsT - topLeftRow , rowsB);

			for (int i=0; i < rowCnt; i++)
			{
				for (int j=0; j < columnCnt; j++)
				{
					target[i + topLeftRow][j + topLeftColumn] = block[i][j];
				}
			}
		}

		/** get subblock of matrix and put it in other matrix
		 */
		template<typename T, int rowsB, int colsB, int rowsT, int colsT>
		FORCE_INLINE static void getBlock(unsigned int topLeftRow, unsigned int topLeftColumn, const MatBase<T, rowsB, colsB>& block, MatBase<T, rowsT, colsT> &target)
		{
			int columnCnt = std::min<int>(colsB - topLeftColumn , colsT);
			int rowCnt = std::min<int>(rowsB - topLeftRow , rowsT);

			for (int i=0; i < rowCnt; i++)
			{
				for (int j=0; j < columnCnt; j++)
				{
					target[i][j] = block[i + topLeftRow][j + topLeftColumn];
				}
			}
		}

		friend std::ostream& operator<< <T, rows, cols>(std::ostream&, const MatBase<T, rows, cols>&);
	};

	template<typename T, int rows, int cols>
	std::ostream& operator <<(std::ostream& s, const MatBase<T, rows, cols>& m)
	{
		for (int i=0; i < rows; i++)
		{
			s << m.v[i] << "\n";
		}
		return s;
	}


}

#endif



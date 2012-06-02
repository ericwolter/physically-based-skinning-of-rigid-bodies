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

#include <Math2/MatrixNxM.h>
#include <cmath>

using namespace IBDS;

/** Standard-Konstruktor: erstellt die Einheitsmatrix
*/
MatrixNxM::MatrixNxM(const unsigned char r, const unsigned char c)
{
	rows = r;
	cols = c;
	for (unsigned char i=0; i < rows; i++)
	{
		v[i].n = c;
	}
}

/** Copy-Konstruktor
  */
MatrixNxM::MatrixNxM (const MatrixNxM& copy)
{
	rows = copy.rows;
	cols = copy.cols;
	for (int i=0; i < rows; i++)
	{
		v[i].n = cols;
		for (int j=0; j < cols; j++)
			v[i].v[j] = copy.v[i].v[j];
	}
}

/** Destruktor
*/
MatrixNxM::~MatrixNxM ()
{ 
	rows = 0;
	cols = 0;
}


/** Negation: -m\n
* Negiert alle Elemente der Matrix
*/
MatrixNxM IBDS::operator - (const MatrixNxM& a)
{ 
	MatrixNxM m = MatrixNxM(a.rows, a.cols);
	for (int i=0; i < a.rows; i++)
		m.v[i] = -a.v[i];
	return m;
}

/** Addition: m1 + m2\n
* Elementweise Addition von m1 mit m2
*/
MatrixNxM IBDS::operator + (const MatrixNxM& a, const MatrixNxM& b)
{ 
	MatrixNxM m = MatrixNxM(a.rows, a.cols);
	for (int i=0; i < a.rows; i++)
		m.v[i] = a.v[i] + b.v[i];
	return m; 
}

/** Subtraktion: m1 - m2\n
* Elementweise Subtraktion 
*/
MatrixNxM IBDS::operator - (const MatrixNxM& a, const MatrixNxM& b)
{ 
	MatrixNxM m = MatrixNxM(a.rows, a.cols);
	for (int i=0; i < a.rows; i++)
		m.v[i] = a.v[i] - b.v[i];
	return m; 
}

/** Multiplikation: m1 * m2\n
* Matrixmultiplikation von m1 mit m2
*/
MatrixNxM IBDS::operator * (const MatrixNxM& a, const MatrixNxM& b) 
{
	MatrixNxM c = MatrixNxM(a.rows, b.cols);
	for (int i=0; i < a.rows; i++)
	{
		for (int j=0; j < b.cols; j++)
		{
			c.v[i].v[j] = 0.0;
			for (int k=0; k < a.cols; k++)
				c.v[i].v[j] += a.v[i].v[k]*b.v[k].v[j];
		}
	}
	return c;
}


/** Multiplikation: d*a\n
* Elementweise Multiplikation einer Matrix mit einer Zahl
*/
MatrixNxM IBDS::operator * (const Real d, const MatrixNxM& a)
{
	MatrixNxM m = MatrixNxM(a.rows, a.cols);
	for (int i=0; i < a.rows; i++)
		m.v[i] = d*a.v[i];
	return m; 
}


/** Transponiert die NxM-Matrix 
*/
MatrixNxM MatrixNxM::transpose () const
{
	MatrixNxM m = MatrixNxM(cols, rows);
	for (int i=0; i < rows; i++)
		for (int j=0; j < cols; j++)
			m.v[j].v[i] = v[i].v[j];
	return m;
}

/** Zugriff per Index auf die einzelnen Komponenten des Vektors.
*/
VectorND& MatrixNxM::operator [] ( int i) 
{
	return v[i];
}

/** Zugriff per Index auf die einzelnen Komponenten des Vektors.
*/
const VectorND& MatrixNxM::operator [] ( int i) const
{
	return v[i];
}


/** Stream-Ausgabe der Matrix
  */
ostream& IBDS::operator << (ostream& s, const MatrixNxM& m)
{ 
	for (int i=0; i < m.rows; i++)
		s << m.v[i] << '\n';
	return s;
}

/** Zugriff per Index auf die einzelnen Komponenten der Matrix.
  */
Real& MatrixNxM::operator () (int i, int j) 
{
	return v[i][j];
}

/** Zugriff per Index auf die einzelnen Komponenten der Matrix.
  */
const Real& MatrixNxM::operator () (int i, int j) const
{
	return v[i][j];
}

/** Gibt die Anzahl der Spalten zurück.
  */
int MatrixNxM::getCols() const
{
	return cols;
}

/** Gibt die Anzahl der Zeilen zurück.
*/
int MatrixNxM::getRows() const
{
	return rows;
}

/** Setzt alle Elemente auf Null. 
*/
void MatrixNxM::zero()
{
	for (int i=0; i < rows; i++)
		for (int j=0; j < cols; j++)
			v[i][j] = 0.0;
}

/** Zuweisung: m1 = m2\n
  * Kopiert die Werte von Matrix m2 in Matrix m1.
  */
MatrixNxM& MatrixNxM::operator = (const MatrixNxM& m)
{ 
	if (v == NULL)
	{
		rows = m.rows;
		cols = m.cols;
		for (int i=0; i < rows; i++)
		{
			v[i].n = cols;
			for (int j=0; j < cols; j++)
				v[i].v[j] = m.v[i].v[j];
		}
	}
	else
	{
		for (int i=0; i < rows; i++)
			for (int j=0; j < cols; j++)
				v[i].v[j] = m.v[i].v[j];
	}
	return *this; 
}

/** Berechnet die Inverse einer Matrix nach Gauss-Jordan. 
  */
bool MatrixNxM::inverse () 
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
			memcpy (ptr, v[row].v, cols * sizeof (double));
			memcpy (v[row].v, v[col].v, cols * sizeof (double));
			memcpy (v[col].v, ptr, cols * sizeof(double));
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

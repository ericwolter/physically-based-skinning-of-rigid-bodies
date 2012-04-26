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

#include "MiniGL.h"

#ifdef WIN32
#include "windows.h"
#endif

#include "GL/gl.h"
#include "GL/glu.h"
#include <GL/freeglut.h>
#include "Common/StringTools.h"

#define _USE_MATH_DEFINES

#include "math.h"

using namespace IBDS;

float MiniGL::fovy = 45;
float MiniGL::znear = 0.5f;
float MiniGL::zfar = 1000;
void (*MiniGL::scenefunc)(void) = NULL;
void (*MiniGL::idlefunc)(void) = NULL;
void (*MiniGL::exitfunc)(void) = NULL;
int MiniGL::idlefunchz = 0;
int MiniGL::time1 = 0;
int MiniGL::time2 = 0;
int MiniGL::width = 0;
int MiniGL::height = 0;
//Mat<float,4,4> MiniGL::transformation = Mat<float,4,4>::getIdentity();
Quaternion MiniGL::m_rotation(1,0,0,0);
Real MiniGL::m_zoom = 1.0;
Vector3D MiniGL::m_translation(true);
float MiniGL::movespeed = 1.0f;
float MiniGL::turnspeed = 0.01f;
int MiniGL::mouse_button = -1;
int MiniGL::mouseFuncButton = -1;
int MiniGL::modifier_key = 0;
int MiniGL::mouse_pos_x_old = 0;
int MiniGL::mouse_pos_y_old = 0;
void (*MiniGL::keyfunc [MAX_KEY_FUNC])(void) = {NULL, NULL};
unsigned char MiniGL::key [MAX_KEY_FUNC] = {0,0};
void (*MiniGL::selectionfunc)(const Vec<int,2>&, const Vec<int,2>&) = NULL;
void (*MiniGL::mousefunc)(int,int) = NULL;
int MiniGL::numberOfKeyFunc = 0;
int MiniGL::winID = 0;
int MiniGL::menuID = 0;
int MiniGL::drawMode = GL_FILL;
bool IBDS::MiniGL::m_breakPointActive = true;
bool IBDS::MiniGL::m_breakPointLoop = false;
Vec<int,2> IBDS::MiniGL::m_selectionStart(-1,-1);


const float IBDS::MiniGL::white[4] = {1.0f, 1.0f, 1.0f, 1.0f};
const float IBDS::MiniGL::red[4] = {1.0f, 0.0f, 0.0f, 1.0f};
const float IBDS::MiniGL::green[4] = {0.0f, 1.0f, 0.0f, 1.0f};
const float IBDS::MiniGL::blue[4] = {0.0f, 0.0f, 1.0f, 1.0f};
const float IBDS::MiniGL::black[4] = {0.0f, 0.0f, 0.0f, 1.0f};
const float IBDS::MiniGL::cyan[4] = {0.0f, 1.0f, 1.0f, 1.0f};
const float IBDS::MiniGL::magenta[4] = {1.0f, 0.0f, 1.0f, 1.0f};
const float IBDS::MiniGL::gray[4] = {0.6f, 0.6f, 0.6f, 1.0f};
const float IBDS::MiniGL::yellow[4] = {1.0f, 1.0f, 0.0f, 1.0f};
const float IBDS::MiniGL::darkRed[4] = {0.5f, 0.0f, 0.0f, 1.0f};
const float IBDS::MiniGL::darkGreen[4] = {0.0f, 0.5f, 0.0f, 1.0f};
const float IBDS::MiniGL::darkblue[4] = {0.0f, 0.0f, 0.5f, 1.0f};
const float IBDS::MiniGL::darkCyan[4] = {0.0f, 0.5f, 0.5f, 1.0f};
const float IBDS::MiniGL::darkMagenta[4] = {0.5f, 0.0f, 0.5f, 1.0f};
const float IBDS::MiniGL::darkYellow[4] = {0.5f, 0.5f, 0.0f, 1.0f};
const float IBDS::MiniGL::darkGray[4] = {0.3f, 0.3f, 0.3f, 1.0f};


void MiniGL::drawEllipsoid(Vector3D& position, const Matrix3x3& ellipsoid, const Real scaleFactor, const float * const color)
{
	float col[] = {1.0,0.0,0.0,1.0};

	glPushMatrix();

	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(color);

	glTranslated(position[0], position[1], position[2]);

	//glRotated(angle, axis.x, axis.y, axis.z);
	//glScaled(ellipsoid[0].length(), ellipsoid[1].length(), ellipsoid[2].length());	
	//glScaled(1.0, 0.1,2.0);	

	GLUquadricObj *quadric;
	quadric = gluNewQuadric();
	gluQuadricDrawStyle(quadric, GLU_FILL );

	Matrix4x4 mat(true);
	mat[0][0] = ellipsoid[0][0];
	mat[0][1] = ellipsoid[0][1];
	mat[0][2] = ellipsoid[0][2];
	mat[0][3] = 0.0;

	mat[1][0] = ellipsoid[1][0];
	mat[1][1] = ellipsoid[1][1];
	mat[1][2] = ellipsoid[1][2];
	mat[1][3] = 0.0;

	mat[2][0] = ellipsoid[2][0];
	mat[2][1] = ellipsoid[2][1];
	mat[2][2] = ellipsoid[2][2];
	mat[2][3] = 0.0;

	mat[3][0] = 0.0;
	mat[3][1] = 0.0;
	mat[3][2] = 0.0;
	mat[3][3] = 1.0;

	glMultMatrixd(&mat[0][0]);

	gluSphere(quadric, scaleFactor, 16, 16);
	//drawSphere2(&position, 1.0, col);
	glPopMatrix();
}

void IBDS::MiniGL::addTweakBarParameter(const char *name, TwType type, TwSetVarCallback setCallback, TwGetVarCallback getCallback, void *clientData, const char *def)
{
	TwAddVarCB(m_tweakBar, name, type, setCallback, getCallback, clientData, def);	
}

void IBDS::MiniGL::addTweakBarButton(const char *name, TwButtonCallback callback, void *clientData, const char *def)
{
	TwAddButton(m_tweakBar, name, callback, clientData, def);
}

void IBDS::MiniGL::setBreakPointActive( const bool active )
{
	m_breakPointActive = active;
}

void IBDS::MiniGL::drawTime( const Real time )
{
	m_time = (float) time;
	TwRefreshBar(m_tweakBar);
}

void IBDS::MiniGL::setSelectionFunc( void (*func) (const Vec<int,2>&, const Vec<int,2>&) )
{
	selectionfunc = func;
}

float IBDS::MiniGL::getZNear()
{
	return znear;
}

float IBDS::MiniGL::getZFar()
{
	return zfar;
}

void IBDS::MiniGL::unproject( const int x, const int y, Vector3D &pos )
{
	GLint viewport[4];
	GLdouble mv[16],pm[16];
	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_MODELVIEW_MATRIX, mv);
	glGetDoublev(GL_PROJECTION_MATRIX, pm);

	GLdouble resx,resy,resz;
	float zNear = MiniGL::getZNear();
	float zFar = MiniGL::getZFar();
	gluUnProject(x, viewport[3] - y, zNear , mv, pm, viewport, &resx, &resy, &resz);
	pos[0] = resx;
	pos[1] = resy;
	pos[2] = resz;
}


/** Zeichnet Vektoren der Koordinaten-Achsen.
  */
void MiniGL::coordinateSystem() 
{
	Vector3D a = Vector3D (0,0,0);
	Vector3D b = Vector3D (2,0,0);
	Vector3D c = Vector3D (0,2,0);
	Vector3D d = Vector3D (0,0,2);

	float diffcolor [4] = {1,0,0,1};
	float speccolor [4] = {1,1,1,1};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, diffcolor);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, diffcolor);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glLineWidth (2);

	glBegin (GL_LINES);
		glVertex3v (&a[0]);
		glVertex3v (&b[0]);
	glEnd ();

	float diffcolor2 [4] = {0,1,0,1};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, diffcolor2);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, diffcolor2);

	glBegin (GL_LINES);
		glVertex3v (&a[0]);
		glVertex3v (&c[0]);
	glEnd ();

	float diffcolor3 [4] = {0,0,1,1};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, diffcolor3);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, diffcolor3);

	glBegin (GL_LINES);
		glVertex3v (&a[0]);
		glVertex3v (&d[0]);
	glEnd ();
	glLineWidth (1);
}

/** Zeichnet eine Linie von a nach b mit der Dicke w und der Farbe farbe.
  */
void MiniGL::drawVector (const Vector3D &a, const Vector3D &b, const float w, const float *color)
{
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(color);

	glLineWidth (w);

	glBegin (GL_LINES);
		glVertex3v(&a[0]);
		glVertex3v(&b[0]);
	glEnd ();
	
	glLineWidth (1);
}

/** Zeichnet eine Linie von (x1,y1,z1) nach (x2,y2,z2) mit der Dicke w und der Farbe farbe.
  */
void MiniGL::drawVector (const Real x1, const Real y1, const Real z1, const Real x2, const Real y2, const Real z2, const float w, float *color)
{
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);

	glLineWidth (w);

	glBegin (GL_LINES);
		glVertex3(x1,y1,z1);
		glVertex3(x2,y2,z2);
	glEnd ();
}

/** Zeichnet eine Kugel an der Stelle translation mit dem übergebenen Radius
* und in der übergebenen Farbe.
*/
void MiniGL::drawSphere (Vector3D *translation, float radius, const float *color, const unsigned int subDivision)
{
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(color);

	glPushMatrix ();
	glTranslated ((*translation)[0], (*translation)[1], (*translation)[2]);
	glutSolidSphere(radius, subDivision, subDivision);
	glPopMatrix ();
}

/** Zeichnet einen Punkt an der Stelle translation mit der übergebenen Größe
  * und in der übergebenen Farbe.
  */
void MiniGL::drawPoint (const Vector3D &translation, const float pointSize, const float * const color)
{	
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(color);

	glPointSize(pointSize);

	glBegin (GL_POINTS);
	glVertex3v(&translation[0]);
	glEnd ();

	glPointSize(1);
}


/** Zeichnet einen Quader an der Stelle translation 
  * in der übergebenen Farbe.
  */
void MiniGL::drawCube (Vector3D *translation, Matrix3x3 *rotation, float width, float height, float depth, float *color)
{
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);

	Real val[16];
	val[0] = width*(*rotation)[0][0]; val[1] = width*(*rotation)[0][1]; val[2] = width*(*rotation)[0][2]; val[3] = 0;
	val[4] = height*(*rotation)[1][0]; val[5] = height*(*rotation)[1][1]; val[6] = height*(*rotation)[1][2]; val[7] = 0;
	val[8] = depth*(*rotation)[2][0]; val[9] = depth*(*rotation)[2][1]; val[10] = depth*(*rotation)[2][2]; val[11] = 0;
	val[12] = (*translation)[0]; val[13] = (*translation)[1]; val[14] = (*translation)[2]; val[15] = 1;

	glPushMatrix ();
	glMultMatrix (val);
	glutSolidCube(1.0);
	glPopMatrix ();
}


/** Zeichnet einen Bitmaptext auf den Bildschirm an die Position x,y
  * (0,0 ist die Mitte des Bildschirms.
  */
void MiniGL::drawBitmapText (float x, float y, const char *str, int strLength, const float *color)
{
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);

	glPushMatrix ();
	glLoadIdentity ();
	glMatrixMode (GL_PROJECTION);
	glPushMatrix ();
	glLoadIdentity ();
	glMatrixMode (GL_MODELVIEW);
	glRasterPos2f (x,y);

	for (int i=0; i < strLength; i++)
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, str[i]);
	glMatrixMode (GL_PROJECTION);
	glPopMatrix ();
	glMatrixMode (GL_MODELVIEW);
	glPopMatrix ();
}

/** Zeichnet einen Stroketext auf den Bildschirm an die Position x,y
  * (0,0 ist die Mitte des Bildschirms.
  */
void MiniGL::drawStrokeText (const Real x, const Real y, const Real z, float scale, const char *str, int strLength, const float *color)
{
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);

	glPushMatrix ();
	glTranslated (x, y, z);
	glScalef (scale, scale, scale);

	for (int i=0; i < strLength; i++)
		glutStrokeCharacter(GLUT_STROKE_ROMAN, str[i]);
	glPopMatrix ();
}

/** Zeichnet einen Stroketext auf den Bildschirm an die Position x,y,z.
  */
void MiniGL::drawStrokeText (const Vector3D &pos, float scale, const char *str, int strLength, const float *color)
{
	drawStrokeText(pos[0], pos[1], pos[2], scale, str, strLength, color);
}


/** Zeichnet ein Rechteck mit den vier Punkten a,b,c,d, der Normalen norm und
  * der übergebenen Farbe color.
  */
void MiniGL::drawQuad (const Vector3D &a, const Vector3D &b, const Vector3D &c, const Vector3D &d, const Vector3D &norm, float *color)
{
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);

	glBegin (GL_QUADS);
		glNormal3v(&norm[0]);
		glVertex3v(&a[0]);
		glVertex3v(&b[0]);
		glVertex3v(&c[0]);
		glVertex3v(&d[0]);
	glEnd ();
}

/** Zeichnet ein Dreieck mit den Eckpunkten a,b,c, der Normalen norm und
  * der Farbe color.
  */
void MiniGL::drawTriangle (const Vector3D &a, const Vector3D &b, const Vector3D &c, const Vector3D &norm, const float *color)
{
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(color);

	glBegin (GL_TRIANGLES);
		glNormal3v(&norm[0]);
		glVertex3v(&a[0]);
		glVertex3v(&b[0]);
		glVertex3v(&c[0]);
	glEnd ();
}

void MiniGL::drawTriangle (const Vector3D &a, const Vector3D &b, const Vector3D &c, const Vector3D &n1, const Vector3D &n2, const Vector3D &n3, const float *color)
{
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(color);

	glBegin (GL_TRIANGLES);
	glNormal3v(&n1[0]);
	glVertex3v(&a[0]);
	glNormal3v(&n2[0]);
	glVertex3v(&b[0]);
	glNormal3v(&n3[0]);
	glVertex3v(&c[0]);
	glEnd ();
}

void MiniGL::drawSubdividedTriangle(const Vector3D &a, const Vector3D &b, const Vector3D &c, const Vector3D &n1, const Vector3D &n2, const Vector3D &n3, const unsigned subdivisions, const float *color)
{
	/* triangle subdivision using vertex numbers */
	if(subdivisions > 0)
	{
		Vector3D v[3];
		v[0] = 0.5*(a + b); 
		v[1] = 0.5*(b + c); 
		v[2] = 0.5*(c + a); 
		Vector3D n[3];
		n[0] = 0.5*(n1 + n2);
		n[1] = 0.5*(n2 + n3);
		n[2] = 0.5*(n3 + n1);
		drawSubdividedTriangle(a, v[0], v[2], n1, n[0], n[2], subdivisions-1, color);
		drawSubdividedTriangle(v[0], b, v[1], n[0], n2, n[1], subdivisions-1, color);
		drawSubdividedTriangle(v[2], v[1], c, n[2], n[1], n3, subdivisions-1, color);
		drawSubdividedTriangle(v[0], v[1], v[2], n[0], n[1], n[2], subdivisions-1, color);
	}
	else
		drawTriangle(a, b, c, n1, n2, n3, color); /* draw triangle at end of recursion */
}

void MiniGL::drawSubdividedTriangle(const Vector3D &a, const Vector3D &b, const Vector3D &c, const unsigned subdivisions, const float *color)
{
	/* triangle subdivision using vertex numbers */
	if(subdivisions > 0)
	{
		Vector3D v[3];
		v[0] = 0.5*(a + b); 
		v[1] = 0.5*(b + c); 
		v[2] = 0.5*(c + a); 
		drawSubdividedTriangle(a, v[0], v[2], subdivisions-1, color);
		drawSubdividedTriangle(v[0], b, v[1], subdivisions-1, color);
		drawSubdividedTriangle(v[2], v[1], c, subdivisions-1, color);
		drawSubdividedTriangle(v[0], v[1], v[2], subdivisions-1, color);
	}
	else
		drawTriangle(a, b, c, color); /* draw triangle at end of recursion */

}

/** Zeichnet ein Dreieck mit den Eckpunkten a,b,c, der Normalen norm und
  * der Farbe color.
  */
void MiniGL::drawTriangle (const Vector3D &a, const Vector3D &b, const Vector3D &c, const float *color)
{
	const IBDS::Vector3D n = (b-a) ^ (c-a);
	drawTriangle(a, b, c, n, color);
}

/** Zeichnet ein Dreieck mit den Eckpunkten a,b,c, der Normalen norm und
  * der Farbe color.
  */
void MiniGL::drawTriangle (const Vector3D &a, const Vector3D &b, const Vector3D &c, const Vector3D &norm, const float *colorA, const float *colorB, const float *colorC)
{
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);

	glBegin (GL_TRIANGLES);
		glNormal3v(&norm[0]);
		glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, colorA);
		glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, colorA);
		glColor4fv(colorA);
		glVertex3v(&a[0]);

		glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, colorB);
		glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, colorB);
		glColor4fv(colorB);
		glVertex3v(&b[0]);

		glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, colorC);
		glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, colorC);
		glColor4fv(colorC);
		glVertex3v(&c[0]);
	glEnd ();
}

/** Draw a tetrahedron.
 */
void MiniGL::drawTetrahedron(const Vector3D &a, const Vector3D &b, const Vector3D &c, const Vector3D &d, const float *color)
{
	Vector3D normal1 = (b-a) ^ (c-a);
	Vector3D normal2 = (b-a) ^ (d-a);
	Vector3D normal3 = (c-a) ^ (d-a);
	Vector3D normal4 = (c-b) ^ (d-b);
	drawTriangle(a, b, c, normal1, color);
	drawTriangle(a, b, d, normal2, color);
	drawTriangle(a, c, d, normal3, color);
	drawTriangle(b, c, d, normal4, color);
}

/** Setzt den Viewport mit dem Winkel pfovy, den z-Ebenen pznear und pzfar,
  * dem Augpunkt peyepoint und dem Punkt plookat.
  */
void MiniGL::setViewport (float pfovy, float pznear, float pzfar, const Vector3D &peyepoint, const Vector3D &plookat)
{
	fovy = pfovy;
	znear = pznear;
	zfar = pzfar;

	glLoadIdentity ();
	gluLookAt (peyepoint [0], peyepoint [1], peyepoint [2], plookat[0], plookat[1], plookat[2], 0, 1, 0);


	Real lookAtMatrix[16];
	glGetRealv (GL_MODELVIEW_MATRIX, &lookAtMatrix[0]);
	
	Mat<Real,4,4> transformation;
	Matrix3x3 rot;
	Vector3D scale;
	//transformation.set(lookAtMatrix);
	transformation.setFromColumnMajor(lookAtMatrix);
	transformation.getTransformation(m_translation, rot, scale);
	m_zoom = scale[0];
	m_rotation.setFromMatrix3x3(rot);

	glLoadIdentity ();
}

/** Setzt den Viewport mit dem Winkel pfovy, den z-Ebenen pznear und pzfar.
  */
void MiniGL::setViewport (float pfovy, float pznear, float pzfar)
{
	fovy = pfovy;
	znear = pznear;
	zfar = pzfar;
}

/** Setzt die Szenen-Funktion, die gezeichnet werden soll, wenn display
  * aufgerufen wird.
  */
void MiniGL::setClientSceneFunc (void  (*func)(void))
{
	scenefunc = func;
}

/** Zeichnet die Szene mit dem aktuellen Viewport.
  */
void MiniGL::display ()
{
	glPolygonMode (GL_FRONT_AND_BACK, drawMode); 
	viewport ();

	if (scenefunc != NULL)
		scenefunc ();

	TwDraw();  // draw the tweak bar(s)
	glutSwapBuffers();
}

/** Initialisiert die OpenGL-Darstellung und meldet die Callback-
  * Funktionen an.
  */
void MiniGL::init (int argc, char **argv, int width, int height, int posx, int posy, char *name)
{
	fovy = 60;
	znear = 0.5f;
	zfar = 1000;

	scenefunc = NULL;

	glutInit (&argc, argv);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	atexit(destroy);

	glutInitWindowSize (width, height);
	glutInitWindowPosition (posx, posy);

	// Initialize AntTweakBar
	// (note that AntTweakBar could also be initialized after GLUT, no matter)
	if( !TwInit(TW_OPENGL, NULL) )
	{
		// A fatal error occured    
		fprintf(stderr, "AntTweakBar initialization failed: %s\n", TwGetLastError());
		exit(1);
	}
	TwWindowSize(width, height);
	initTweakBar();

	winID = glutCreateWindow (name);

	glEnable (GL_DEPTH_TEST);
	glEnable (GL_NORMALIZE);
	glShadeModel (GL_SMOOTH);
	glEnable (GL_BLEND); 
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glClearColor (0.95f, 0.95f, 1.0f, 1.0f);

	glutReshapeFunc (reshape);
	glutKeyboardFunc (keyboard);
	glutMouseFunc (mousePress);
	glutMotionFunc (mouseMove);
	glutSpecialFunc (special);
	glutDisplayFunc (display);
	glutIdleFunc (idlefunc);
	glutMouseWheelFunc(mouseWheel);

	// after GLUT initialization
	// directly redirect GLUT events to AntTweakBar
	glutPassiveMotionFunc((GLUTmousemotionfun)TwEventMouseMotionGLUT); // same as MouseMotion

	// send the ''glutGetModifers'' function pointer to AntTweakBar
	TwGLUTModifiersFunc(glutGetModifiers);

	glutCreateMenu(processMenuEvents);
	glutAddMenuEntry("Wireframe", MENU_WIREFRAME);
	glutAddMenuEntry("Exit", MENU_EXIT);
	glutAttachMenu(GLUT_RIGHT_BUTTON);

	glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
}

TwBar *MiniGL::m_tweakBar = NULL;
float MiniGL::m_time = 0.0f;
float MiniGL::m_quat[4] = { 0.0f, 0.0f, 0.0f, 1.0f };

TwBar *MiniGL::getTwBar()
{
	return m_tweakBar;
}

void MiniGL::initTweakBar()
{
	// Create a tweak bar
	m_tweakBar = TwNewBar("TweakBar");
	TwDefine(" GLOBAL help='MiniGL TweakBar.' "); // Message added to the help bar.
	TwDefine(" TweakBar size='250 500' position='5 5' color='96 200 224' text=dark "); // change default tweak bar size and color

	TwAddVarRO(m_tweakBar, "Time", TW_TYPE_FLOAT, &m_time, " label='Time' precision=5 group=General");

	TwAddVarCB(m_tweakBar, "Rotation", TW_TYPE_QUAT4F, setRotationCB, getRotationCB, &m_quat, 
		" label='Rotation' open help='Change the rotation.' group=General ");

	// Add callback to toggle auto-rotate mode (callback functions are defined above).
	TwAddVarCB(m_tweakBar, "Wireframe", TW_TYPE_BOOL32, setWireframeCB, getWireframeCB, NULL, 
		" label='Wireframe' key=w help='Toggle wireframe mode.' group=General ");
}

void TW_CALL MiniGL::setWireframeCB(const void *value, void *clientData)
{
	const int val = *(const int *)(value);
	if (val == 0) 
		drawMode = GL_FILL;
	else 
		drawMode = GL_LINE;
}

void TW_CALL MiniGL::getWireframeCB(void *value, void *clientData)
{
	*(int *)(value) = drawMode == GL_LINE;
}

void TW_CALL MiniGL::setRotationCB(const void *value, void *clientData)
{
	const float *val = (const float *)(value);
	m_rotation[1] = (Real) val[0];
	m_rotation[2] = (Real) val[1];
	m_rotation[3] = (Real) val[2];
	m_rotation[0] = (Real) val[3];

}

void TW_CALL MiniGL::getRotationCB(void *value, void *clientData)
{
	float *val = (float*)(value);
	val[0] = (float) m_rotation[1];
	val[1] = (float) m_rotation[2];
	val[2] = (float) m_rotation[3];
	val[3] = (float) m_rotation[0];
}

void MiniGL::cleanupTweakBar()
{

}

/** Wird aufgerufen, wenn das Programm beendet wird und das Glut-Fenster geschlossen werden soll.
  */
void MiniGL::destroy ()
{
	TwTerminate();
	//glutDestroyWindow (winID);
}

/** Wird aufgerufen, wenn sich die Höhe oder die Breite des Fensters
  * geändert hat.
  */
void MiniGL::reshape (int w, int h)
{
	if ((w > 0) && (h > 0))
	{
		width = w;
		height = h;
		glutReshapeWindow (w,h);

		TwWindowSize(width, height);
		glutPostRedisplay ();
	}
}

/** Übergibt die Idle-Funktion, die höchstens in der übergebenen Frequenz hz
  * aufgerufen wird.
  \see idle
  */
void MiniGL::setClientIdleFunc (int hz, void (*func) (void))
{
	if ((hz == 0) || (func == NULL))
	{
		idlefunchz = 0;
		idlefunc = NULL;
		glutIdleFunc (NULL);
	}
	else
	{
		idlefunchz = (int) ((1.0/hz)*1000.0);
		idlefunc = func;
		glutIdleFunc (idle);
	}
}

/** Übergibt eine Keyboard-Funktion, die aufgerufen wird, wenn k gedrückt wurde.
  */
void MiniGL::setKeyFunc (int nr, unsigned char k, void (*func) (void))
{
	if ((nr >= MAX_KEY_FUNC) || (func == NULL))
	{
		return;
	}
	else
	{
		keyfunc[nr] = func;
		key[nr] = k;
		numberOfKeyFunc++;
	}
}


/** Übergibt eine Maus-Funktion, die aufgerufen wird, wenn die Maus bewegt wird und der Button gedrückt ist. 
  */
void MiniGL::setMouseMoveFunc(int button, void (*func) (int,int))
{
	mousefunc = func;
	mouseFuncButton = button;
}


/** Führt die aktuelle Idle-Funktion unter Berücksichtigung der aktuellen
  * Frequenz aus.
  \see setClientIdleFunc
  */
void MiniGL::idle ()
{
	idlefunc ();
	glutPostRedisplay ();
}

/** Callback-Funktion für Keyboard-Eingaben. \n\n
  * Ermöglicht eine Steuerung der Translation in z-Richtung und der Rotation mittels 
  * der Tasten a, z bzw. 1,...,6.\n\n
  * a - vorwärts\n
  * z - rückwärts\n
  * 1 - neg. Drehung um x-Achse\n
  * 2 - pos. Drehung um x-Achse\n
  * 3 - neg. Drehung um y-Achse\n
  * 4 - pos. Drehung um y-Achse\n
  * 5 - neg. Drehung um z-Achse\n
  * 6 - pos. Drehung um z-Achse
  \see special
  */
void MiniGL::keyboard (unsigned char k, int x, int y)
{
	if (TwEventKeyboardGLUT(k, x, y))  // send event to AntTweakBar
		return;

	if (k == 27)
	{
		m_breakPointLoop = false;
		m_breakPointActive = false;
		glutLeaveMainLoop();
		return;
	}
	else if (k == 97)
		move (0, 0, movespeed);
	else if (k == 121)
		move (0, 0, -movespeed);
	else if (k == 49)
		rotateX(-turnspeed);
	else if (k == 50)
		rotateX(turnspeed);
	else if (k == 51)
		rotateY(-turnspeed);
	else if (k == 52)
		rotateY(turnspeed);
// 	else if (k == 53)
// 		anglez -= turnspeed;
// 	else if (k == 54)
// 		anglez += turnspeed;
	else 
	{
		for (int i=0; i < numberOfKeyFunc; i++)
		{
			if (k == key[i])
				keyfunc [i] ();
		}
	}
	glutPostRedisplay ();
}

/** Callback-Funktion für Menü-Events. Wird aufgerufen, wenn 
  * der Benutzer einen Menü-Punkt im Kontextmenü ausgewählt hat.
  */
void MiniGL::processMenuEvents(int option) 
{
	switch (option) 
	{
		case MENU_WIREFRAME : 	{
								if (drawMode == GL_LINE)
									drawMode = GL_FILL;
								else 
									drawMode = GL_LINE;
								glutPostRedisplay ();
								break;
					}
		case MENU_EXIT :	{
								exitfunc ();
								destroy ();
								exit (0);
								break;
							}
	}
}


/** Callback-Funktion für Spezial-Keyboard-Eingaben. 
  * Ermöglicht eine Steuerung der Translation in x- und in y-Richtung mittels 
  * der Cursor-Tasten.
  \see keyboard
  */
void MiniGL::special (int k, int x, int y)
{
	if (TwEventSpecialGLUT(k, x, y))  // send event to AntTweakBar
		return;

	if (k == GLUT_KEY_UP)
		move (0, -movespeed, 0);
	else if (k == GLUT_KEY_DOWN)
		move (0, movespeed, 0);
	else if (k == GLUT_KEY_LEFT)
		move (movespeed, 0, 0);
	else if (k == GLUT_KEY_RIGHT)
		move (-movespeed, 0, 0);
	else if (k == GLUT_KEY_F5)
		m_breakPointLoop = false;
	glutPostRedisplay ();
}

void MiniGL::setProjectionMatrix (int width, int height) 
{ 
	glMatrixMode(GL_PROJECTION); 
	glLoadIdentity(); 
	gluPerspective (fovy, (float)width/(float)height, znear, zfar); 
}

/** Setzt den Viewport mit den aktuellen Parametern.
  */
void MiniGL::viewport ()
{
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glRenderMode (GL_RENDER);
	glViewport (0, 0, width, height);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
	setProjectionMatrix (width, height);
	glMatrixMode (GL_MODELVIEW);

	glTranslatef((float) m_translation[0], (float) m_translation[1], (float) m_translation[2]);
	Matrix3x3 rot;
	m_rotation.getMatrix3x3(rot);
	Mat<Real,4,4> transform;
	Vector3D scale(m_zoom, m_zoom, m_zoom);
	transform.setTransformation(m_translation, rot, scale);
	Real transformMatrix[16];
	//transform.get(&transformMatrix[0]);
	transform.getColumnMajor(&transformMatrix[0]);
	glLoadMatrix(&transformMatrix[0]);
}

/** Initialisiert die Lichtquellen für OpenGL.
  */
void MiniGL::initLights ()
{
	float t = 0.9f;
	float a = 0.2f;
	float amb0 [4] = {a,a,a,1};
	float diff0 [4] = {t,0,0,1};
	float spec0 [4] = {1,1,1,1};
	float pos0 [4] = {-10,10,10,1};
	glLightfv(GL_LIGHT0, GL_AMBIENT,  amb0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE,  diff0);
	glLightfv(GL_LIGHT0, GL_SPECULAR, spec0);
	glLightfv(GL_LIGHT0, GL_POSITION, pos0);
	glEnable(GL_LIGHT0);

	float amb1 [4] = {a,a,a,1};
	float diff1 [4] = {0,0,t,1};
	float spec1 [4] = {1,1,1,1};
	float pos1 [4] = {10,10,10,1};
	glLightfv(GL_LIGHT1, GL_AMBIENT,  amb1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE,  diff1);
	glLightfv(GL_LIGHT1, GL_SPECULAR, spec1);
	glLightfv(GL_LIGHT1, GL_POSITION, pos1);
	glEnable(GL_LIGHT1);

	float amb2 [4] = {a,a,a,1};
	float diff2 [4] = {0,t,0,1};
	float spec2 [4] = {1,1,1,1};
	float pos2 [4] = {0,10,10,1};
	glLightfv(GL_LIGHT2, GL_AMBIENT,  amb2);
	glLightfv(GL_LIGHT2, GL_DIFFUSE,  diff2);
	glLightfv(GL_LIGHT2, GL_SPECULAR, spec2);
	glLightfv(GL_LIGHT2, GL_POSITION, pos2);
	glEnable(GL_LIGHT2);


	glEnable(GL_LIGHTING);
	glLightModeli (GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

}

/** Bewegt die Kamera im lokalen Koordinatensystem um die Verschiebung (x,y,z)
  */
void MiniGL::move (float x, float y, float z)
{
	m_translation[0] += x;
	m_translation[1] += y;
	m_translation[2] += z;
}

/** Bewegt die Kamera im lokalen Koordinatensystem um den Mittelpunkt der Szene.
  */
void MiniGL::rotateY (float y)
{
	Quaternion quat;
	quat.setFromAxisAngle(Vector3D(0,1,0), y);
	m_rotation = m_rotation*quat;
}

/** Bewegt die Kamera im lokalen Koordinatensystem um den Mittelpunkt der Szene.
  */
void MiniGL::rotateX (float x)
{
	Quaternion quat;
	quat.setFromAxisAngle(Vector3D(1,0,0), x);
	m_rotation = m_rotation*quat;
}


/** Wird aufgerufen, wenn ein Mouse-Pressed-Event ausgelöst wird.
  * Speichert den aktuellen Status.
  */
void MiniGL::mousePress (int button, int state, int x, int y)
{
	if (TwEventMouseButtonGLUT(button, state, x, y))  // send event to AntTweakBar
		return;

	if (state == GLUT_DOWN)
		mouse_button = button;
	else 
		mouse_button = -1;
	modifier_key = glutGetModifiers ();

	mouse_pos_x_old = x;
	mouse_pos_y_old = y;

	if (selectionfunc != NULL)
	{
		if (button == GLUT_LEFT_BUTTON) 
		{
			if (state == GLUT_DOWN)
				m_selectionStart = Vec<int,2>(x,y);
			else
			{
				if (m_selectionStart[0] != -1)
				{
					const Vec<int,2> pos(x,y);
					selectionfunc(m_selectionStart, pos);
				}
				m_selectionStart = Vec<int,2>(-1,-1);
			}
		}
	}

	glutPostRedisplay ();
}

void MiniGL::mouseWheel(int button, int dir, int x, int y)
{
	if (dir > 0)
	{
		//move (0, 0, movespeed);
		movespeed *= 2.0;
	}
	else
	{
		//move (0, 0, -movespeed);
		movespeed *= 0.5;
	}
}


/** Wird aufgerufen, wenn ein Mouse-Move-Event ausgelöst wird.
  * Verschiebt bzw. rotiert die Szene.
  */
void MiniGL::mouseMove (int x, int y)
{
	if (TwEventMouseMotionGLUT(x, y))  // send event to AntTweakBar
		return;

	int d_x = mouse_pos_x_old - x;
	int d_y = y - mouse_pos_y_old;

	if (mouse_button == GLUT_LEFT_BUTTON)
	{
		// translate scene in z direction		
		if (modifier_key == GLUT_ACTIVE_CTRL)
		{
			move (0, 0, -(d_x + d_y) / 10.0f);
		}
		// translate scene in x/y direction
		else if (modifier_key == GLUT_ACTIVE_SHIFT)
		{
			move (-d_x / 20.0f, -d_y / 20.0f, 0);
		}
		// rotate scene around x, y axis
		else if (modifier_key == GLUT_ACTIVE_ALT)
		{
			rotateX(d_y/ 100.0f);
			rotateY(-d_x/ 100.0f);
		}
	}
	if (mousefunc != NULL)
	{
		if ((mouseFuncButton == -1) || (mouseFuncButton == mouse_button))
			mousefunc(x, y);
	}

	mouse_pos_x_old = x;
	mouse_pos_y_old = y;

	glutPostRedisplay ();
}

void MiniGL::rotate( float x, float y, float z )
{
	//float DEG_TO_RAD = (float) (M_PI / 180.0);
	Quaternion quatX;
	quatX.setFromAxisAngle(Vector3D(1,0,0), x);
	Quaternion quatY;
	quatY.setFromAxisAngle(Vector3D(0,1,0), y);
	Quaternion quatZ;
	quatZ.setFromAxisAngle(Vector3D(0,0,1), z);
	
	m_rotation = m_rotation*quatX*quatY*quatZ;
}

void MiniGL::breakPoint()
{
	glutPostRedisplay ();
	breakPointMainLoop();
}

void MiniGL::breakPointMainLoop()
{
	if (m_breakPointActive)
	{
		m_breakPointLoop = true;
		while( m_breakPointLoop )
		{
			glutMainLoopEvent( );
		}
	}
}

void MiniGL::drawCylinder(const Vector3D &a, const Vector3D &b, const float *color, const float radius, const unsigned int subdivisions)
{
	drawCylinder(a[0], a[1], a[2], b[0], b[1], b[2], color, radius, subdivisions);
}


/**
* Renders a closed cylinder between two points.
*
* Based on the following implementation by Joel J. Parris:
* http://home.neo.rr.com/jparris/OpenGL%20-%20draw%20cylinder%20between%202%20pts.htm
*
* @param x1 the x coordinate of the first point.
* @param y1 the y coordinate of the first point.
* @param z1 the z coordinate of the first point.
* @param x2 the x coordinate of the second point.
* @param y2 the y coordinate of the second point.
* @param z2 the z coordinate of the second point.
* @param radius the radius of the cylinder.
* @param subdivisions the number of subdivisions to use to represent the cylinder (number of "strips" on the sides and number of "slices" on the top and bottom disks)
*/
void MiniGL::drawCylinder(const Real x1, const Real y1, const Real z1, const Real x2, const Real y2, const Real z2, const float *color, const float radius, const unsigned int subdivisions)
{
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(color);

	float vx = (float) (x2-x1);
	float vy = (float) (y2-y1);
	float vz = (float) (z2-z1);
	//handle the degenerate case with an approximation
	if(vz == 0)
		vz = .00000001f;
	float v = sqrt( vx*vx + vy*vy + vz*vz );
	float ax = 57.2957795f*acos( vz/v );
	if ( vz < 0.0 )
		ax = -ax;
	float rx = -vy*vz;
	float ry = vx*vz;

	GLUquadricObj *quadric=gluNewQuadric();
	gluQuadricNormals(quadric, GLU_SMOOTH);

	glPushMatrix();
	glTranslatef((float) x1, (float) y1, (float) z1 );
	glRotatef(ax, rx, ry, 0.0);
	//draw the cylinder
	gluCylinder(quadric, radius, radius, v, subdivisions, 1);
	gluQuadricOrientation(quadric,GLU_INSIDE);
	//draw the first cap
	gluDisk( quadric, 0.0, radius, subdivisions, 1);
	glTranslatef( 0,0,v );
	//draw the second cap
	gluQuadricOrientation(quadric,GLU_OUTSIDE);
	gluDisk( quadric, 0.0, radius, subdivisions, 1);
	glPopMatrix();

	gluDeleteQuadric(quadric);
}
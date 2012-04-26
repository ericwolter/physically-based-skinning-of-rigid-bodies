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

#ifndef __MINIGL_H__
#define __MINIGL_H__

//#define TW_NO_LIB_PRAGMA //anttweakbar shall not try to link AntTweakBar64.lib

#include "Common/Config.h"
#include "Math2/Vec.h"
#include "Math2/Mat.h"
#include "anttweakbar/include/AntTweakBar.h"
#include "Math2/Quaternion.h"

#ifdef USE_DOUBLES
	#define glNormal3v glNormal3dv
	#define glVertex3v glVertex3dv
	#define glVertex3 glVertex3d
	#define glMultMatrix glMultMatrixd
	#define glGetRealv glGetDoublev
	#define glLoadMatrix glLoadMatrixd
#else
	#define glNormal3v glNormal3fv
	#define glVertex3v glVertex3fv
	#define glVertex3 glVertex3f
	#define glMultMatrix glMultMatrixf
	#define glGetRealv glGetFloatv
	#define glLoadMatrix glLoadMatrixf
#endif


namespace IBDS
{
	/** Klasse mit ein paar Befehlen für die Darstellung mit
	  * OpenGL.
	  \author Jan Bender
	  */
	class MiniGL
	{
	#define MAX_KEY_FUNC 30

	public:
		static const float white[4];
		static const float red[4];
		static const float green[4];
		static const float blue[4];
		static const float black[4];
		static const float cyan[4];
		static const float magenta[4];
		static const float gray[4];
		static const float yellow[4];
		static const float darkRed[4];
		static const float darkGreen[4];
		static const float darkblue[4];
		static const float darkCyan[4];
		static const float darkMagenta[4];
		static const float darkYellow[4];
		static const float darkGray[4];

	private:
		enum
		{
			MENU_WIREFRAME = 1,
			MENU_EXIT
		};

	
		/** Öffnungswinkel der Kamera */
		static float fovy;
		/** Vorder z-Ebene */
		static float znear;
		/** Hintere z-Ebene */
		static float zfar;
		/** Funktion zum Zeichnen der Szene */
		static void (*scenefunc)(void);
		/** Funktion, die beim Verlassen des Programms aufgerufen wird */
		static void (*exitfunc)(void);
		/** Idle-Funktion */
		static void (*idlefunc)(void);
		/** Tastenfunktionen */
		static void (*keyfunc [MAX_KEY_FUNC])(void);
		/** Mausfunktion */
		static void (*mousefunc)(int,int);
		/** selection callback */
		static void (*selectionfunc) (const Vec<int,2>&, const Vec<int,2>&);
		/** Tasten, die die Tastenfunktionen auslösen */
		static unsigned char key [MAX_KEY_FUNC];
		/** Anzahl der Tastenfunktionen */
		static int numberOfKeyFunc;
		static int mouseFuncButton;
		/** Frequenz mit der die Idle-Funktion ausgeführt werden soll */
		static int idlefunchz;
		/** Bildschirmbreite */
		static int width;
		/** Bildschirmhöhe */
		static int height;
		/** Zeitpunkt 1 */
		static int time1;
		/** Zeitpunkt 2 */
		static int time2;
// 		/** Transformation matrix */
// 		static Mat<float,4,4> transformation;
		static Vector3D m_translation;
		static Quaternion m_rotation;
		static Real m_zoom;
		/** Geschwindigkeit */
		static float movespeed;
		/** Drehgeschwindigkeit */
		static float turnspeed;
		/** Gedrückter Mausknopf */
		static int mouse_button;
		/** Modifier (Shift, Ctrl, Alt) */
		static int modifier_key;
		/** Letzte Mausposition */
		static int mouse_pos_x_old;
		/** Letzte Mausposition */
		static int mouse_pos_y_old;
		/** Gibt an, wie die Szene gezeichnet werden soll. (Wireframe) */
		static int drawMode;
		static Vec<int,2> m_selectionStart;
		static bool m_breakPointActive;
		static bool m_breakPointLoop;

		static void processMenuEvents(int option);
		static void reshape (int w, int h);
		static void idle ();
		static void keyboard (unsigned char k, int x, int y);
		static void special (int k, int x, int y);
		static void mousePress (int button, int state, int x, int y);
		static void mouseMove (int x, int y);
		static void mouseWheel(int button, int dir, int x, int y);
		static int winID;
		static int menuID;
		static void breakPointMainLoop();
		static void drawElements();
		
	public:
		static void coordinateSystem ();
		static void drawVector (const Vector3D &a, const Vector3D &b, const float w, const float *color);
		static void drawVector (const Real x1, const Real y1, const Real z1, const Real x2, const Real y2, const Real z2, const float w, float *color);
		static void drawSphere (Vector3D *translation, float radius, const float *color, const unsigned int subDivision =  16);
		static void drawQuad (const Vector3D &a, const Vector3D &b, const Vector3D &c, const Vector3D &d, const Vector3D &norm, float *color);
		static void drawTetrahedron(const Vector3D &a, const Vector3D &b, const Vector3D &c, const Vector3D &d, const float *color);
		static void drawTriangle (const Vector3D &a, const Vector3D &b, const Vector3D &c, const Vector3D &norm, const float *color);
		static void drawTriangle (const Vector3D &a, const Vector3D &b, const Vector3D &c, const Vector3D &n1, const Vector3D &n2, const Vector3D &n3, const float *color);
		static void drawTriangle (const Vector3D &a, const Vector3D &b, const Vector3D &c, const float *color);
		static void drawTriangle (const Vector3D &a, const Vector3D &b, const Vector3D &c, const Vector3D &norm, const float *colorA, const float *colorB, const float *colorC);
		static void drawSubdividedTriangle(const Vector3D &a, const Vector3D &b, const Vector3D &c, const Vector3D &n1, const Vector3D &n2, const Vector3D &n3, const unsigned subdivisions, const float *color);
		static void drawSubdividedTriangle(const Vector3D &a, const Vector3D &b, const Vector3D &c, const unsigned subdivisions, const float *color);
		static void drawBitmapText (float x, float y, const char *str, int strLength, const float *color);
		static void drawStrokeText (const Real x, const Real y, const Real z, float scale, const char *str, int strLength, const float *color);
		static void drawStrokeText (const Vector3D &pos, float scale, const char *str, int strLength, const float *color);
		static void drawCube (Vector3D *translation, Matrix3x3 *rotation, float width, float height, float depth, float *color);		
		static void drawPoint (const Vector3D &translation, const float pointSize, const float * const color);
		static void setViewport (float pfovy, float pznear, float pzfar, const Vector3D &peyepoint, const Vector3D &plookat);
		static void setViewport (float pfovy, float pznear, float pzfar);
		static void setClientSceneFunc (void (*func)(void));
		static void display ();
		static void setClientIdleFunc (int hz, void (*func) (void));
		static void setKeyFunc (int nr, unsigned char k, void (*func) (void));
		static void setMouseMoveFunc (int button, void (*func) (int, int));
		static void setSelectionFunc (void (*func) (const Vec<int,2>&, const Vec<int,2>&));
		static void init (int argc, char **argv, int width, int height, int posx, int posy, char *name);
		static void destroy ();
		static void viewport ();
		static void initLights ();
		static void move (float x, float y, float z);
		static void rotateX (float x);
		static void rotateY (float y);
		static void rotate(float x, float y, float z);
		static void setProjectionMatrix (int width, int height);
		static void setBreakPointActive(const bool active);
		static void breakPoint();
		static void clearPoints();
		static void clearLines();
		static void clearTriangles();
		static void clearElements();
		static void addPoint(const Vector3D &a, const float pointSize, const float *color);
		static void addLine(const Vector3D &a, const Vector3D &b, const float lineWidth, const float *color);
		static void addTriangle(const Vector3D &a, const Vector3D &b, const Vector3D &c, const float *color);
		static void drawTime(const Real time);		
		static void drawEllipsoid(Vector3D& position, const Matrix3x3& ellipsoid, const Real scaleFactor, const float * const color);
		static void drawCylinder(const Real x1, const Real y1, const Real z1, const Real x2, const Real y2, const Real z2,  const float *color, const float radius=0.02, const unsigned int subdivisions=8);
		static void drawCylinder(const Vector3D &a, const Vector3D &b,  const float *color, const float radius=0.02, const unsigned int subdivisions=8);
		static void unproject(const int x, const int y, Vector3D &pos);

		static float getZNear();
		static float getZFar();


	private:
		static TwBar *m_tweakBar;
		static float m_time;
		static float m_quat[4];

		static void initTweakBar();
		static void cleanupTweakBar();
		static void TW_CALL setWireframeCB(const void *value, void *clientData);
		static void TW_CALL getWireframeCB(void *value, void *clientData);		
		static void TW_CALL setRotationCB(const void *value, void *clientData);
		static void TW_CALL getRotationCB(void *value, void *clientData);	
		
	public:
		static TwBar *getTwBar();
		static void addTweakBarParameter(const char *name, TwType type, TwSetVarCallback setCallback, TwGetVarCallback getCallback, void *clientData, const char *def);
		static void addTweakBarButton(const char *name, TwButtonCallback callback, void *clientData, const char *def);
	};
}

#endif

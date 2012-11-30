#include <math.h>
#include <stdlib.h>
#include <iostream>

#include "main.h"
#include "constants.h"

// angle of rotation for the camera direction
float cam_angle=0.0;
// actual vector representing the camera's direction
float cam_lx=0.0f,cam_lz=-1.0f;
// XZ position of the camera
float cam_x=0.0f,cam_z=30.0f;
// the key states. These variables will be zero
//when no key is being presses
float deltaAngle = 0.0f;
float deltaMove = 0;

float elapsed = 0;

bool breakpoint = false;

int main(int argc, char **argv) {
    
    cout << "time" << "\t" << "err_x" << "\t" << "err_y" << "\t" << "err_z" << endl;
	
	simulation = new Simulation();
	simulation->init(false);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH|GLUT_DOUBLE|GLUT_RGBA);
    glutInitWindowSize(800,600);
    glutInitWindowPosition(50, 25);
    glutCreateWindow("Thesis");
	
	glutDisplayFunc(timeStep);
	glutReshapeFunc(reshape);
	glutIdleFunc(timeStep);
	glutKeyboardFunc(keyboard);
	
	glutSpecialFunc(specialKeyPress);
	glutIgnoreKeyRepeat(1);
	glutSpecialUpFunc(specialKeyRelease);
	
	glEnable(GL_DEPTH_TEST);
	
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
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
	
    glutMainLoop();
    return 0;
}

void reshape(GLint w, GLint h) {
	
	if (h == 0) {
		h = 1;
	}
	
	float ratio = 1.0 * w / h;
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0, 0, w, h);
	gluPerspective(45, ratio, 1, 1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void keyboard(unsigned char key, int x, int y) {
	
	if (key == 27) {
		exit(0);
	} else if (key == 32) {
        breakpoint = breakpoint ? false : true;
    }
}

void specialKeyPress(int key, int x, int y) {
	
	switch (key) {
		case GLUT_KEY_LEFT : deltaAngle = -0.01f; break;
		case GLUT_KEY_RIGHT : deltaAngle = 0.01f; break;
		case GLUT_KEY_UP : deltaMove = 0.5f; break;
		case GLUT_KEY_DOWN : deltaMove = -0.5f; break;
	}
}

void specialKeyRelease(int key, int x, int y) {

	switch (key) {
		case GLUT_KEY_LEFT :
		case GLUT_KEY_RIGHT : deltaAngle = 0.0f;break;
		case GLUT_KEY_UP :
		case GLUT_KEY_DOWN : deltaMove = 0;break;
	}
}

void timeStep() {
	
	if (deltaMove) {
		cam_x += deltaMove * cam_lx * 0.1f;
		cam_z += deltaMove * cam_lz * 0.1f;
	}
	if (deltaAngle) {
		cam_angle += deltaAngle;
		cam_lx = sin(cam_angle);
		cam_lz = -cos(cam_angle);
	}
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glLoadIdentity();
	gluLookAt(cam_x, 1.0f, cam_z,
			cam_x+cam_lx, 1.0f,  cam_z+cam_lz,
			0.0f, 1.0f,  0.0f);
    
    if(!breakpoint) {
        // logging for graph output
        if(int(elapsed * 1/stepSize) % 10 == 0) {
            btVector3 err = simulation->block->getCenterOfMassPosition() - btVector3(0.0f,3.5f,0.0f);
            cout << elapsed << "\t" << err.x() << "\t" << err.y() << "\t" << err.z() << endl;
        }

        simulation->step(stepSize);
        elapsed += stepSize;
    }
    
    if(elapsed > 10) {
        breakpoint = true;
    }
	simulation->render();
	
	glutSwapBuffers();
}


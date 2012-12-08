#include <math.h>
#include <stdlib.h>
#include <iostream>

#include "main.h"
#include "constants.h"


// polar coordinates of camera
float cam_radius = 20.0f;
float cam_phi = M_PI_2; // 90 degrees;
float cam_theta = M_PI_2; // 90 degrees;

float cam_speed = 2.0f;

float eye_x = 0.0f;
float eye_y = 0.0f;
float eye_z = 0.0f;

float center_x = 0.0f;
float center_y = 0.0f;
float center_z = 0.0f;

btVector3 rest;

float elapsed = 0;
bool breakpoint = true;

int main(int argc, char **argv) {
    
    cout << "time" << "\t" << "err_x" << "\t" << "err_y" << "\t" << "err_z" << endl;
	
	simulation = new Simulation();
	simulation->init(body_type);
    
    center_x = simulation->block->getCenterOfMassPosition().x();
    center_y = simulation->block->getCenterOfMassPosition().y();
    center_z = simulation->block->getCenterOfMassPosition().z();
    
    rest = simulation->block->getCenterOfMassPosition();

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH|GLUT_DOUBLE|GLUT_RGBA);
    glutInitWindowSize(800,600);
    glutInitWindowPosition(50, 25);
    glutCreateWindow("Thesis");
	
	glutDisplayFunc(timeStep);
	glutReshapeFunc(reshape);
	glutIdleFunc(timeStep);
	glutKeyboardFunc(keyboard);
	
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
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(65.0, (float)w / (float)h, 0.01, 100);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void keyboard(unsigned char key, int x, int y) {
	
	if (key == 27) {
		exit(0);
	} else if (key == 32) {
        breakpoint = breakpoint ? false : true;
    } else if (key == 'w') {
        center_z += 0.2f * cam_speed;
    } else if (key == 's') {
        center_z -= 0.2f * cam_speed;
    } else if (key == 'd') {
        center_x -= 0.2f * cam_speed;
    } else if (key == 'a') {
        center_x += 0.2f * cam_speed;
    } else if (key == 'e') {
        center_y += 0.2f * cam_speed;
    } else if (key == 'q') {
        center_y -= 0.2f * cam_speed;
    } else if (key == 'i') {
        cam_theta -= 0.0174532925f * cam_speed; // 1 degree;
    } else if (key == 'k') {
        cam_theta += 0.0174532925f * cam_speed; // 1 degree;
    } else if (key == 'l') {
        cam_phi -= 0.0174532925f * cam_speed; // 1 degree;
    } else if (key == 'j') {
        cam_phi += 0.0174532925f * cam_speed; // 1 degree;
    } else if (key == 'u') {
        cam_radius += 0.2f * cam_speed;
    } else if (key == 'o') {
        cam_radius -= 0.2f * cam_speed;
    }
}

void timeStep() {
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
    eye_x = center_x + sinf(cam_theta) * cos(cam_phi) * cam_radius;
    eye_z = center_z + sinf(cam_theta) * sinf(cam_phi) * cam_radius;
    eye_y = center_y + cosf(cam_theta) * cam_radius;
    
	glLoadIdentity();
	gluLookAt(eye_x, eye_y, eye_z,
			center_x, center_y,  center_y,
			0.0f, 1.0f,  0.0f);
    
    if(!breakpoint) {
        // logging for graph output
        if(int(elapsed * 1/stepSize) % 100 == 0) {
            btVector3 err = simulation->block->getCenterOfMassPosition() - rest;
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


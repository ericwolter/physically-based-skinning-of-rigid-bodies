#ifndef __Thesis__main__
#define __Thesis__main__

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include <GL/glut.h>
#endif

#include "simulation.h"

int WIDTH;
int HEIGHT;

int main(int argc, char **argv);

void display(void);
void reshape(GLint w, GLint h);
void keyboard(unsigned char key, int x, int y);
void specialKeyPress(int key, int x, int y);
void specialKeyRelease(int key, int x, int y);

void timeStep();

Simulation *simulation;

#endif /* defined(__Thesis__main__) */
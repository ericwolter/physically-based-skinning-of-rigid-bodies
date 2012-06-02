#include <cstdlib>
#include <iostream>

#include <GL/glfw.h>
#include <AntTweakBar.h>

void init();
void loop();
void draw();
void draw_square(float red, float green, float blue);
void shutdown();

TwBar *m_tweakBar = NULL;
float m_time = 0.0f;

float rotate_y = 0;
float rotate_z = 0;
const float rotations_per_tick = 0.002;
const int squares = 15;

int main( int argc, char **argv )
{
	std::cout << "test";
	init();
	loop();
	shutdown();
}

void init() {
	std::cout << "initalizing...";
	const int window_width = 800;
	const int window_height = 600;

	if (!glfwInit()) {
		exit(EXIT_FAILURE);
	} else {
		TwInit(TW_OPENGL, NULL);
	}

	if (!glfwOpenWindow(window_width,window_height,0,0,0,0,8,0, GLFW_WINDOW)) {
		glfwTerminate();
		exit(EXIT_FAILURE);
	} else {
		glfwSetWindowTitle("Rigid Body");

		TwWindowSize(window_width, window_height);

		m_tweakBar = TwNewBar("TweakBar");
		TwDefine(" GLOBAL help='MiniGL TweakBar.' "); // Message added to the help bar.
		//TwDefine(" TweakBar size='250 500' position='5 5' color='96 200 224' text=dark "); // change default tweak bar size and color

		TwAddVarRO(m_tweakBar, "Time", TW_TYPE_FLOAT, &m_time, " label='Time' precision=5 group=General");

		// TwAddVarCB(m_tweakBar, "Rotation", TW_TYPE_QUAT4F, setRotationCB, getRotationCB, &m_quat, " label='Rotation' open help='Change the rotation.' group=General ");

		// // Add callback to toggle auto-rotate mode (callback functions are defined above).
		// TwAddVarCB(m_tweakBar, "Wireframe", TW_TYPE_BOOL32, setWireframeCB, getWireframeCB, NULL, " label='Wireframe' key=w help='Toggle wireframe mode.' group=General ");

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
  		float aspect_ratio = ((float)window_height) / window_width;
  		glFrustum(.5, -.5, -.5 * aspect_ratio, .5 * aspect_ratio, 1, 50);
  		glMatrixMode(GL_MODELVIEW);
	}
}

void loop() {
	double old_time = glfwGetTime();

	while(true) {
		double current_time = glfwGetTime();
		double delta_rotate = (current_time + old_time) * rotations_per_tick * 360;

		old_time = current_time;

		if(glfwGetKey(GLFW_KEY_ESC) == GLFW_PRESS) {
			break;
		}
		if(glfwGetKey(GLFW_KEY_LEFT) == GLFW_PRESS) {
			rotate_y += delta_rotate;
		}
		if(glfwGetKey(GLFW_KEY_RIGHT) == GLFW_PRESS) {
			rotate_y -= delta_rotate;
		}

		rotate_z += delta_rotate;

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		draw();
		TwDraw();

		glfwSwapBuffers();
	}
}

void draw() {
	glLoadIdentity();
	glTranslatef(0,0,-30);
	glRotatef(rotate_y, 0, 1, 0);
	// glRotatef(rotate_z, 0, 0, 1);

	float red = 0;
	float blue = 1;

	for (int i = 0; i < squares; ++i) {
		glRotatef(360.0/squares, 0, 0, 1);
		red += 1.0/12;
		blue -= 1.0/12;
		draw_square(red, .6, blue);
	}

}

void draw_square(float red, float green, float blue) {
	glBegin(GL_QUADS);
	{
		glColor3f(red, green, blue);
		glVertex2i(1, 11);
		glColor3f(red * .8, green * .8, blue * .8);
    	glVertex2i(-1, 11);
    	glColor3f(red * .5, green * .5, blue * .5);
    	glVertex2i(-1, 9);
    	glColor3f(red * .8, green * .8, blue * .8);
    	glVertex2i(1, 9);
	}
	glEnd();
}

void shutdown() {
	glfwTerminate();
	exit(EXIT_SUCCESS);
}
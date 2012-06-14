#include <cstdlib>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <math.h>

#define GLFW_INCLUDE_GL3
#define GLFW_NO_GLU
#include <GL/glfw.h>

#include "ObjLoader.h"
#include "Helper.h"
#include "RigidBody.h"

#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

void init();
void initProgram();
void initVertexBuffer();
void initVertexArrayObjects();

void loop();
void keyboard();
void draw();
void draw_square(float red, float green, float blue);
void shutdown();
void resize(int width, int height);

Vector3f resolveCameraPosition();
Affine3f calculateLookAtMatrix(const Vector3f &cameraPoint, const Vector3f &lookPoint);
Affine3f perspectiveMatrixFunc(const float &fovy, const float &aspect, const float &zNear, const float &zFar);

int windowWidth = 800;
int windowHeight = 600;
float g_fzNear = 1.0f;
float g_fzFar = 1000.0f;
string base_path("");
ObjLoader objLoader;
Helper helper;
GLuint program;
RigidBody cube;
RigidBody cube2;

static Vector3f cameraTarget;
static Vector3f cameraRelativePosition;
static Vector4f lightDirection;

GLuint dirToLightUnif;
GLuint lightIntensityUnif;

GLuint cameraToClipMatrixUnif;
GLuint modelToCameraMatrixUnif;
GLuint normalModelToCameraMatrixUnif;

GLuint vertexBufferObject;
GLuint indexBufferObject;
GLuint vaoObject;

GLuint planeBufferObject;
GLuint planeNormalBufferObject;
GLuint planeVertexArray;

// Only works for unix based systems
string ExtractDirectory(const string &path)
{
	return path.substr(0, path.find_last_of( '/' ) + 1);
}

int main(int argc, const char *argv[])
{
	base_path = ExtractDirectory(argv[0]);

	init();
	loop();
	shutdown();
}

void init()
{
	if (!glfwInit())
	{
		exit(EXIT_FAILURE);
	}
	else
	{
		glfwOpenWindowHint(GLFW_OPENGL_VERSION_MAJOR, 3);
		glfwOpenWindowHint(GLFW_OPENGL_VERSION_MINOR, 2);
		glfwOpenWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
		glfwOpenWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	}

	if (!glfwOpenWindow(windowWidth, windowHeight, 0, 0, 0, 0, 8, 0, GLFW_WINDOW))
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	else
	{
		glfwSetWindowSizeCallback(resize);
		glfwSetWindowTitle("Rigid Body");

		cameraTarget << 0.0f, 0.0f, 0.0f;
		cameraRelativePosition << 90.0f, -20, 20.f;
		lightDirection << 0.0f, 0.5f, 0.0f, 0.0f;

		initProgram();
		initVertexBuffer();
		initVertexArrayObjects();

		cube = RigidBody(objLoader.parseObj(base_path + "models/cube.obj"), Vector3f::Zero());
		cube2 = RigidBody(objLoader.parseObj(base_path + "models/cube.obj"), (MatrixXf(3,1) << 1.0f, 5.0f, 0.0f).finished());

		// glEnable(GL_CULL_FACE);
		// glCullFace(GL_BACK);
		// glFrontFace(GL_CW);

		glEnable(GL_DEPTH_TEST);
		glDepthMask(GL_TRUE);
		glDepthFunc(GL_LEQUAL);
		glDepthRange(0.0f, 1.0f);
		glEnable(GL_DEPTH_CLAMP);
	}
}

void initProgram()
{
	std::vector<GLuint> shaders;
	shaders.push_back(helper.loadShader(GL_VERTEX_SHADER, base_path + "shaders/PosOnlyWorldTransform.vert"));
	shaders.push_back(helper.loadShader(GL_FRAGMENT_SHADER, base_path + "shaders/ColorPassthrough.frag"));

	program = helper.createProgram(shaders);
	std::for_each(shaders.begin(), shaders.end(), glDeleteShader);

	cameraToClipMatrixUnif = glGetUniformLocation(program, "cameraToClipMatrix");
	modelToCameraMatrixUnif = glGetUniformLocation(program, "modelToCameraMatrix");
	normalModelToCameraMatrixUnif = glGetUniformLocation(program, "normalModelToCameraMatrix");
	dirToLightUnif = glGetUniformLocation(program, "dirToLight");
	lightIntensityUnif = glGetUniformLocation(program, "lightIntensity");

	resize(windowWidth, windowHeight);
}

const float planeNormal[] =
{
	0.0f, 1.0f, 0.0f,
	0.0f, 1.0f, 0.0f,
	0.0f, 1.0f, 0.0f,

	0.0f, 1.0f, 0.0f,
	0.0f, 1.0f, 0.0f,
	0.0f, 1.0f, 0.0f,
};

const float planeData[] =
{
	-5.0f, 0.0f, -5.0f, 1.0f,
	5.0f, 0.0f, 5.0f, 1.0f,
	-5.0f, 0.0f, 5.0f, 1.0f,

	-5.0f, 0.0f, -5.0f, 1.0f,
	5.0f, 0.0f, -5.0f, 1.0f,
	5.0f, 0.0f, 5.0f, 1.0f,

	0.0f, 40.0f/255.0f, 50.0f/255.0f, 1.0f,
	0.0f, 40.0f/255.0f, 50.0f/255.0f, 1.0f,
	0.0f, 40.0f/255.0f, 50.0f/255.0f, 1.0f,

	0.0f, 40.0f/255.0f, 50.0f/255.0f, 1.0f,
	0.0f, 40.0f/255.0f, 50.0f/255.0f, 1.0f,
	0.0f, 40.0f/255.0f, 50.0f/255.0f, 1.0f
};

void initVertexBuffer()
{
	glGenBuffers(1, &planeBufferObject);
	glBindBuffer(GL_ARRAY_BUFFER, planeBufferObject);
	glBufferData(GL_ARRAY_BUFFER, sizeof(planeData), planeData, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glGenBuffers(1, &planeNormalBufferObject);
	glBindBuffer(GL_ARRAY_BUFFER, planeNormalBufferObject);
	glBufferData(GL_ARRAY_BUFFER, sizeof(planeNormal), planeNormal, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void initVertexArrayObjects()
{
	glGenVertexArrays(1, &planeVertexArray);
	glBindVertexArray(planeVertexArray);

	size_t colorData = sizeof(planeData) / 2;
	glBindBuffer(GL_ARRAY_BUFFER, planeBufferObject);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, (void *)colorData);

	glBindBuffer(GL_ARRAY_BUFFER, planeNormalBufferObject);
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindVertexArray(0);
}

void loop()
{
	double old_time = glfwGetTime();

	while (true)
	{
		double current_time = glfwGetTime();
		old_time = current_time;

		if (glfwGetKey(GLFW_KEY_ESC) == GLFW_PRESS)
		{
			break;
		}
		keyboard();

		draw();

		// cube.integrate(0.001);
		cube2.integrate(0.01);

		glfwSwapBuffers();
	}
}

void keyboard()
{
	if (glfwGetKey('W') == GLFW_PRESS)
	{
		cameraTarget(2) -= 1.0f;
	}
	if (glfwGetKey('S') == GLFW_PRESS)
	{
		cameraTarget(2) += 1.0f;
	}
	if (glfwGetKey('D') == GLFW_PRESS)
	{
		cameraTarget(0) -= 1.0f;
	}
	if (glfwGetKey('A') == GLFW_PRESS)
	{
		cameraTarget(0) += 1.0f;
	}
	if (glfwGetKey('E') == GLFW_PRESS)
	{
		cameraTarget(1) -= 1.0f;
	}
	if (glfwGetKey('Q') == GLFW_PRESS)
	{
		cameraTarget(1) += 1.0f;
	}

	if (glfwGetKey('I') == GLFW_PRESS)
	{
		cameraRelativePosition(1) -= 1.0f;
	}
	if (glfwGetKey('K') == GLFW_PRESS)
	{
		cameraRelativePosition(1) += 1.0f;
	}
	if (glfwGetKey('J') == GLFW_PRESS)
	{
		cameraRelativePosition(0) -= 1.0f;
	}
	if (glfwGetKey('L') == GLFW_PRESS)
	{
		cameraRelativePosition(0) += 1.0f;
	}
	if (glfwGetKey('O') == GLFW_PRESS)
	{
		cameraRelativePosition(2) -= 1.0f;
	}
	if (glfwGetKey('U') == GLFW_PRESS)
	{
		cameraRelativePosition(2) += 1.0f;
	}    
}

void draw()
{
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	const Vector3f cameraPosition = resolveCameraPosition();
	Affine3f cameraTransform;
	cameraTransform = calculateLookAtMatrix(cameraPosition, cameraTarget);

	glUseProgram(program);

	Vector4f lightDirectionInCameraSpace;
	lightDirectionInCameraSpace = cameraTransform.matrix() * lightDirection;

	glUniform3f(dirToLightUnif, lightDirectionInCameraSpace[0], lightDirectionInCameraSpace[1], lightDirectionInCameraSpace[2]);
	glUniformMatrix3fv(normalModelToCameraMatrixUnif, 1, GL_FALSE, cameraTransform.matrix().block<3,3>(0,0).data());
	glUniform4f(lightIntensityUnif, 1.0f, 1.0f, 1.0f, 1.0f);
	
	cube.render(cameraTransform, modelToCameraMatrixUnif);
	cube2.render(cameraTransform, modelToCameraMatrixUnif);

	cameraTransform.scale(10.0f);
	glUniformMatrix4fv(modelToCameraMatrixUnif, 1, GL_FALSE, cameraTransform.data());
	glBindVertexArray(planeVertexArray);
	glDrawArrays(GL_TRIANGLES, 0, 8);

	glUseProgram(0);
}

void shutdown()
{
	glfwTerminate();
	exit(EXIT_SUCCESS);
}

void resize(int w, int h)
{
	Affine3f t;
	t = perspectiveMatrixFunc(45.0, (w / (float)h), g_fzNear, g_fzFar);

	glUseProgram(program);
	glUniformMatrix4fv(cameraToClipMatrixUnif, 1, GL_FALSE, t.data());
	glUseProgram(0);

	glViewport(0, 0, (GLsizei) w, (GLsizei) h);
}

// extracted from glm: https://github.com/Groovounet/glm/blob/master/glm/gtc/matrix_transform.inl
Affine3f perspectiveMatrixFunc(const float &fovy, const float &aspect, const float &zNear, const float &zFar)
{
	float range = tan((fovy / 2.0f) * M_PI / 180.0f) * zNear;
	float left = -range * aspect;
	float right = range * aspect;
	float bottom = -range;
	float top = range;

	Matrix4f result = Matrix4f::Zero();
	result(0, 0) = (2.0f * zNear) / (right - left);
	result(1, 1) = (2.0f * zNear) / (top - bottom);
	result(2, 2) = -(zFar + zNear) / (zFar - zNear);
	result(2, 3) = -1.0f;
	result(3, 2) = -(2.0f * zFar * zNear) / (zFar - zNear);

	Affine3f t;
	t = result;
	return t;
}

Vector3f resolveCameraPosition()
{
	float phi = cameraRelativePosition(0) * (M_PI / 180.0f);
	// cout << "phi: " << phi << endl;
	float theta = (cameraRelativePosition(1) + 90.0f) * (M_PI / 180.0f);
	// cout << "theta: " << theta << endl;

	Vector3f directionToCamera(sin(theta) * cos(phi), cos(theta), sin(theta) * sin(phi));
	// cout << "directionToCamera: " << directionToCamera << endl;
	return (directionToCamera * cameraRelativePosition(2)) + cameraTarget;
}

Affine3f calculateLookAtMatrix(const Vector3f &cameraPoint, const Vector3f &lookPoint)
{
	// cout << "input cameraPoint: " << cameraPoint << endl;
	// cout << "input lookPoint: " << lookPoint << endl;
	const Vector3f upDirection(0.0f, 1.0f, 0.0f);

	Vector3f lookDirection = (lookPoint - cameraPoint).normalized();
	// cout << "lookDirection: " << lookDirection << endl;
	Vector3f rightDirection = lookDirection.cross(upDirection).normalized();
	// cout << "rightDirection: " << rightDirection << endl;
	Vector3f perpendicularUpDirection = rightDirection.cross(lookDirection);
	// cout << "perpendicularUpDirection: " << perpendicularUpDirection << endl;

	Matrix4f rotationMatrix = Matrix4f::Identity();
	// cout << "rotationMatrix: " << rotationMatrix << endl;
	rotationMatrix.col(0) << (MatrixXf(4, 1) << rightDirection, 0.0f).finished();
	rotationMatrix.col(1) << (MatrixXf(4, 1) << perpendicularUpDirection, 0.0f).finished();
	rotationMatrix.col(2) << (MatrixXf(4, 1) << -lookDirection, 0.0f).finished();
	// cout << "rotationMatrix_filled: " << rotationMatrix << endl;
	rotationMatrix.transposeInPlace();
	// cout << "rotationMatrix_transpose: " << rotationMatrix << endl;

	Matrix4f translationMatrix = Matrix4f::Identity();
	translationMatrix.col(3) << (MatrixXf(4, 1) << -cameraPoint, 1.0f).finished();
	// cout << "translationMatrix: " << translationMatrix << endl;

	Affine3f t;
	t = rotationMatrix * translationMatrix;
	return t;
}

//
//  main.cpp
//  BachelorThesis
//
//  Created by Eric Wolter on 9/13/12.
//  Copyright (c) 2012 Eric Wolter. All rights reserved.
//

#include <vector>
using namespace std;

#define GLFW_INCLUDE_GL3
#define GLFW_NO_GLU
#include <GL/glfw.h>

#include <Eigen/Dense>
using namespace Eigen;

#include "Helper.h"
#include "Simulation.h"

// constants
// window
const int g_windowWidth = 640;
const int g_windowHeight = 480;
// opengl
const int g_zNear = 1.0f;
const int g_zFar = 1000.0f;

// globals
GLuint program;
GLuint cameraToClipMatrixUnif;
GLuint modelToCameraMatrixUnif;
GLuint normalModelToCameraMatrixUnif;
GLuint dirToLightUnif;
GLuint lightIntensityUnif;
GLuint ambientIntensityUnif;

static Vector3f cameraTarget;
static Vector3f cameraRelativePosition;
static Vector4f lightDirection;

Simulation simulation;

// method definitions
void init();
void loop();
void draw();
void keyboard();
void resize(int width, int height);

void initProgram();

Vector3f resolveCameraPosition();
Affine3f calculateLookAtMatrix(const Vector3f &cameraPoint, const Vector3f &lookPoint);
Affine3f perspectiveMatrixFunc(const float &fovy, const float &aspect, const float &zNear, const float &zFar);

void initProgram()
{
    vector<GLuint> shaders;
    shaders.push_back(Helper::loadShader(GL_VERTEX_SHADER, "vertex.vert"));
    shaders.push_back(Helper::loadShader(GL_FRAGMENT_SHADER, "fragment.frag"));

    program = Helper::createProgram(shaders);
    for_each(shaders.begin(), shaders.end(), glDeleteShader);
    
    cameraToClipMatrixUnif = glGetUniformLocation(program, "cameraToClipMatrix");
    modelToCameraMatrixUnif = glGetUniformLocation(program, "modelToCameraMatrix");
    normalModelToCameraMatrixUnif = glGetUniformLocation(program, "normalModelToCameraMatrix");
    dirToLightUnif = glGetUniformLocation(program, "dirToLight");
    lightIntensityUnif = glGetUniformLocation(program, "lightIntensity");
    ambientIntensityUnif = glGetUniformLocation(program, "ambientIntensity");

    resize(g_windowWidth, g_windowHeight);
}

void resize(int width, int height)
{
    Affine3f t;
    t = perspectiveMatrixFunc(45.0, (width / (float)height), g_zNear, g_zFar);
    
    glUseProgram(program);
    glUniformMatrix4fv(cameraToClipMatrixUnif, 1, GL_FALSE, t.data());
    glUseProgram(0);
    
    glViewport(0, 0, (GLsizei)width, (GLsizei)height);
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
    
    if(!glfwOpenWindow(g_windowWidth, g_windowHeight, 0, 0, 0, 0, 8, 0, GLFW_WINDOW))
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    else
    {
        glfwSetWindowTitle("BachelorThesis");
        
        cameraTarget = Vector3f(0.0f, 0.0f, 0.0f);
        cameraRelativePosition = Vector3f(0.0f, 20.0f, -20.0f);
        lightDirection = Vector4f(0.8666f, 0.5f, 0.0f, 0.0f);
        
        initProgram();
        
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glFrontFace(GL_CCW);
        
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LEQUAL);
        glDepthRange(0.0f, 1.0f);
        glEnable(GL_DEPTH_CLAMP);
        
        simulation.init();
    }
}

int main()
{
//    init();
//    loop();
    Particle *p1 = new Particle();
    p1->mass = 0.01f;
    p1->radii = Vector3f(0.2f, 0.1f, 0.1f);
    p1->position = Vector3f(0.0f, 0.0f, 0.0f);
    p1->orientation = AngleAxis<float>(0, Vector3f(1, 0, 0));
    
    Particle *p2 = new Particle();
    p2->mass = 0.01f;
    p2->radii = Vector3f(0.2f, 0.1f, 0.1f);
    p2->position = Vector3f(1.0f, 0.0f, 0.0f);
    p2->orientation = AngleAxis<float>(0, Vector3f(1, 0, 0));
    
    Particle *p3 = new Particle();
    p3->mass = 0.01f;
    p3->radii = Vector3f(0.2f, 0.1f, 0.1f);
    p3->position = Vector3f(1.0f, 1.0f, 0.0f);
    p3->orientation = AngleAxis<float>(0, Vector3f(1, 0, 0));
    
    Particle *p4 = new Particle();
    p4->mass = 0.01f;
    p4->radii = Vector3f(0.2f, 0.1f, 0.1f);
    p4->position = Vector3f(0.0f, 1.0f, 0.0f);
    p4->orientation = AngleAxis<float>(0, Vector3f(1, 0, 0));
    
    ParticleGroup *g = new ParticleGroup();
    g->m_particles.push_back(p1);
    g->m_particles.push_back(p2);
    g->m_particles.push_back(p3);
    g->m_particles.push_back(p4);
    
    g->init();
    
    p1->predictedPosition = Vector3f(-1.0f, 0.0f, 0.0f);
    p2->predictedPosition = Vector3f(1.0f, 0.0f, 0.0f);
    p3->predictedPosition = Vector3f(1.0f, 1.0f, 0.0f);
    p4->predictedPosition = Vector3f(0.0f, 1.0f, 0.0f);
    
    g->update();
    
    return 0;
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
        
//        simulation.step(0.001f);
        
        glfwSwapBuffers();
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
    
    Matrix3f normMatrix = cameraTransform.matrix().block<3,3>(0,0);
    
    glUniformMatrix3fv(normalModelToCameraMatrixUnif, 1, GL_FALSE, normMatrix.inverse().transpose().data());
    glUniform4f(lightIntensityUnif, 0.8f, 0.8f, 0.8f, 1.0f);
    glUniform4f(ambientIntensityUnif, 0.2f, 0.2f, 0.2f, 1.0f);
    
    simulation.render(cameraTransform, modelToCameraMatrixUnif);
    
    glUseProgram(0);
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
    float phi = cameraRelativePosition.x() * (M_PI / 180.0f);
    float theta = (cameraRelativePosition.y() + 90.0f) * (M_PI / 180.0f);
    
    Vector3f directionToCamera(sin(theta) * cos(phi), cos(theta), sin(theta) * sin(phi));
    return (directionToCamera * cameraRelativePosition.z()) + cameraTarget;
}

Affine3f calculateLookAtMatrix(const Vector3f &cameraPoint, const Vector3f &lookPoint)
{
    const Vector3f upDirection(0.0f, 1.0f, 0.0f);
    
    Vector3f lookDirection = (lookPoint - cameraPoint).normalized();
    Vector3f rightDirection = lookDirection.cross(upDirection).normalized();
    Vector3f perpendicularUpDirection = rightDirection.cross(lookDirection);
    
    Matrix4f rotationMatrix = Matrix4f::Identity();
    rotationMatrix.col(0) << (MatrixXf(4, 1) << rightDirection, 0.0f).finished();
    rotationMatrix.col(1) << (MatrixXf(4, 1) << perpendicularUpDirection, 0.0f).finished();
    rotationMatrix.col(2) << (MatrixXf(4, 1) << -lookDirection, 0.0f).finished();
    rotationMatrix.transposeInPlace();
    
    Matrix4f translationMatrix = Matrix4f::Identity();
    translationMatrix.col(3) << (MatrixXf(4, 1) << -cameraPoint, 1.0f).finished();
    
    Affine3f t;
    t = rotationMatrix * translationMatrix;
    return t;
}
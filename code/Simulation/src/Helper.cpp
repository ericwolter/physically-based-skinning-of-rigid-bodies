#include "Helper.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

using namespace std;

Helper::Helper() {}
Helper::~Helper() {}

GLuint Helper::loadShader(GLenum shaderType, const string &shaderFilename)
{
    ifstream shaderFile(shaderFilename.c_str());
    if (shaderFile.is_open())
    {
        stringstream shaderData;
        shaderData << shaderFile.rdbuf();
        shaderFile.close();

        return this->compileShader(shaderType, shaderData.str());
    }
    else
    {
        cout << "Unable to open shader file" << endl;
    }

    return -1;
}

GLuint Helper::compileShader(GLenum shaderType, const std::string &shaderData)
{
    GLuint shader = glCreateShader(shaderType);

    const GLchar *cstrShaderData = shaderData.c_str();
    const GLint cstrLength = shaderData.length();

    glShaderSource(shader, 1, &cstrShaderData, &cstrLength);
    glCompileShader(shader);

    GLint status;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    if (status == GL_FALSE)
    {
        GLint infoLogLength;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLogLength);

        GLchar *strInfoLog = new GLchar[infoLogLength + 1];
        glGetShaderInfoLog(shader, infoLogLength, NULL, strInfoLog);

        const char *strShaderType = NULL;
        switch (shaderType)
        {
        case GL_VERTEX_SHADER: strShaderType = "vertex"; break;
        case GL_GEOMETRY_SHADER: strShaderType = "geometry"; break;
        case GL_FRAGMENT_SHADER: strShaderType = "fragment"; break;
        }

        fprintf(stderr, "Compile failure in %s shader:\n%s\n", strShaderType, strInfoLog);
        delete[] strInfoLog;
    }

    return shader;
}

GLuint Helper::createProgram(const std::vector<GLuint> &shaders)
{
    GLuint program = glCreateProgram();

    vector<GLuint>::const_iterator shader;
    for (shader = shaders.begin(); shader != shaders.end(); shader++)
    {
        glAttachShader(program, *shader);
    }

    glLinkProgram(program);

    GLint status;
    glGetProgramiv (program, GL_LINK_STATUS, &status);
    if (status == GL_FALSE)
    {
        GLint infoLogLength;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &infoLogLength);

        GLchar *strInfoLog = new GLchar[infoLogLength + 1];
        glGetProgramInfoLog(program, infoLogLength, NULL, strInfoLog);
        fprintf(stderr, "Linker failure: %s\n", strInfoLog);
        delete[] strInfoLog;
    }

    for (shader = shaders.begin(); shader != shaders.end(); shader++)
    {
        glDetachShader(program, *shader);
    }

    return program;
}
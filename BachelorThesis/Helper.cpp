//
//  Helper.cpp
//  BachelorThesis
//
//  Created by Eric Wolter on 9/13/12.
//  Copyright (c) 2012 Eric Wolter. All rights reserved.
//

#include "Helper.h"

#include <iostream>
#include <fstream>
#include <sstream>

Helper::Helper() {}
Helper::~Helper() {}

GLuint Helper::loadShader(GLenum shaderType, const std::string &shaderFilename)
{
    std::ifstream shaderFile(shaderFilename.c_str());
    if (shaderFile.is_open())
    {
        std::stringstream shaderData;
        shaderData << shaderFile.rdbuf();
        shaderFile.close();
        
        return Helper::compileShader(shaderType, shaderData.str());
    }
    else
    {
        std::cout << "Unable to open shader file" << std::endl;
    }
    
    return -1;
}

GLuint Helper::compileShader(GLenum shaderType, const std::string &shaderData)
{
    GLuint shader = glCreateShader(shaderType);
    
    const GLchar *cstrShaderData = shaderData.c_str();
    const GLint cstrLength = (int)shaderData.length();
    
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
    
    std::vector<GLuint>::const_iterator shader;
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
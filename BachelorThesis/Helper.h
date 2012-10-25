//
//  Helper.h
//  BachelorThesis
//
//  Created by Eric Wolter on 9/13/12.
//  Copyright (c) 2012 Eric Wolter. All rights reserved.
//

#ifndef __BachelorThesis__Helper__
#define __BachelorThesis__Helper__

#include <string>
#include <vector>

#define GLFW_INCLUDE_GL3
#define GLFW_NO_GLU
#include <GL/glfw.h>

class Helper
{
public:
    Helper();
	~Helper();
    
    static GLuint loadShader(GLenum shaderType, const std::string &shaderFilename);
	static GLuint createProgram(const std::vector<GLuint> &shaderList);
private:
	static GLuint compileShader(GLenum shaderType, const std::string &shaderData);
};

#endif /* defined(__BachelorThesis__Helper__) */

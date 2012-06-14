#ifndef __HELPER_H__
#define __HELPER_H__

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

	GLuint loadShader(GLenum shaderType, const std::string &shaderFilename);
	GLuint createProgram(const std::vector<GLuint> &shaderList);
private:
	GLuint compileShader(GLenum shaderType, const std::string &shaderData);
};

#endif
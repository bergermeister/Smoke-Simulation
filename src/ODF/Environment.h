/**
 * @file Environment.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Environment_h
#define ODF_Environment_h

// OpenGL Includes
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#define GL_GLEXT_PROTOTYPES
#ifdef _WIN32
#define GLUT_DISABLE_ATEXIT_HACK
#include <GL/glew.h>
#endif
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#ifdef _WIN32
#include "GL/wglew.h"
#endif
#endif

// StdLib Includes
#include <cstdint>
#include <cstring>
#include <cassert>
#include <iostream>
#include <fstream>

#endif



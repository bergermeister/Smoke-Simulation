#ifndef _GL_CANVAS_H_
#define _GL_CANVAS_H_

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <string>

// Included files for OpenGL Rendering
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

#include "boundingbox.h"
class ArgParser;
class Camera;
class Smoke;
class RayTracer;
class BoundingBox;
class Mesh;
// ====================================================================
// NOTE:  All the methods and variables of this class are static
// ====================================================================

class GLCanvas {

public:

  // Set up the canvas and enter the rendering loop
  // Note that this function will not return but can be
  // terminated by calling 'exit(0)'
  static void initialize(ArgParser *_args, Mesh *_mesh);
  static void Render();

private:

  static void InitLight();
  static void Load();

  // various static variables
  static Mesh *mesh;
  static ArgParser *args;
  static Camera *camera;
  static Smoke *smoke;
  static RayTracer *raytracer;
  static BoundingBox bbox;

  // state of the mouse cursor
  static int mouseButton;
  static int mouseX;
  static int mouseY;
  static bool shiftPressed;
  static bool controlPressed;
  static bool altPressed;
  static int raytracing_x;
  static int raytracing_y;
  static int raytracing_skip;

  // Callback functions for mouse and keyboard events
  static void display(void);
  static void reshape(int w, int h);
  static void mouse(int button, int state, int x, int y);
  static void motion(int x, int y);
  static void keyboard(unsigned char key, int x, int y);
  static void idle();

  static int DrawPixel();
   static Vec3f TraceRay(double i, double j);
};

// ====================================================================

int HandleGLError(const std::string &message = "");

#endif

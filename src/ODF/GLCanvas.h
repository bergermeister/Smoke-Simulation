/**
 * @file GLCanvas.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Primitive_GLCanvas_h
#define ODF_Primitive_GLCanvas_h

// ODF Includes
#include <ODF/Environment.h>
#include <ODF/Primitive/Mesh.h>

namespace ODF
{
   class GLCanvas 
   {
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

   int HandleGLError( const std::string& Message = "" );
}

#endif


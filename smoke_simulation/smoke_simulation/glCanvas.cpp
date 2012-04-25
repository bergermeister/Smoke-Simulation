#include "glCanvas.h"
#include "argparser.h"
#include "camera.h"
#include "smoke.h"
#include "matrix.h"
#include "raytracer.h"
#include "face.h"
#include "mesh.h"
#include "utils.h"

// ========================================================
// static variables of GLCanvas class

ArgParser* GLCanvas::args = NULL;
Camera* GLCanvas::camera = NULL;
Smoke* GLCanvas::smoke = NULL;
BoundingBox GLCanvas::bbox;
RayTracer* GLCanvas::raytracer = NULL;
Mesh* GLCanvas::mesh = NULL;

int GLCanvas::mouseButton = 0;
int GLCanvas::mouseX = 0;
int GLCanvas::mouseY = 0;

bool GLCanvas::controlPressed = false;
bool GLCanvas::shiftPressed = false;
bool GLCanvas::altPressed = false;


// params for the raytracing animation
int GLCanvas::raytracing_x;
int GLCanvas::raytracing_y;
int GLCanvas::raytracing_skip;

// ========================================================
// Initialize all appropriate OpenGL variables, set
// callback functions, and start the main event loop.
// This function will not return but can be terminated
// by calling 'exit(0)'
// ========================================================

void GLCanvas::initialize(ArgParser *_args, Mesh *_mesh) {
 
  args = _args;
  smoke = NULL;
  raytracer = NULL;
  mesh = _mesh;
  Vec3f point_of_interest = Vec3f(0,0,0);
  Vec3f camera_position = Vec3f(0,0,5);

  Vec3f up = Vec3f(0,1,0);
  camera = new PerspectiveCamera(camera_position, point_of_interest, up, 20 * M_PI/180.0);

   // if not initialized, position a perspective camera and scale it so it fits in the window
 // camera = new PerspectiveCamera();  

  // setup glut stuff
  glutInitWindowSize(args->width, args->height);
  glutInitWindowPosition(100,100);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB);
  glutCreateWindow("OpenGL Viewer");
  HandleGLError("in glcanvas initialize");

#ifdef _WIN32
  GLenum err = glewInit();
  if (err != GLEW_OK) {
      fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
      exit(1);
  }
#endif
  // basic rendering 
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  glShadeModel(GL_SMOOTH);
  glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
  GLfloat ambient[] = { 0.2, 0.2, 0.2, 1.0 };
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

  glCullFace(GL_BACK);
  glDisable(GL_CULL_FACE);

  // Initialize callback functions
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutIdleFunc(idle);

  HandleGLError("finished glcanvas initialize");

  Load();
  bbox.initializeVBOs();

  HandleGLError("finished glcanvas initialize");
  
	mesh->initializeVBOs();
	mesh->setupVBOs();
    
  // Enter the main rendering loop
  glutMainLoop();
}

void GLCanvas::Load() { 
  delete smoke;
  smoke = NULL;
  if(args->smoke_file != "")
  { 
	
		smoke = new Smoke(args);
		RayTracer *_raytracer = new RayTracer(smoke,args,mesh);
		raytracer = _raytracer;
	
		camera=NULL;
		if(camera==NULL){
			bbox.Set(smoke->getBoundingBox());
			Vec3f point_of_interest;
			bbox.getCenter(point_of_interest);
			double max_dim = bbox.maxDim();
			Vec3f camera_position = point_of_interest + Vec3f(0,0,5*max_dim);
			Vec3f up = Vec3f(0,1,0);
			camera = new PerspectiveCamera(camera_position, point_of_interest, up, 20 * M_PI/180.0);  
		}
  }
  if (smoke) smoke->setupVBOs();
}


void GLCanvas::InitLight() {
  // Set the last component of the position to 0 to indicate
  // a directional light source

  GLfloat position[4] = { 30,30,100, 1};
  GLfloat diffuse[4] = { 0.75,0.75,0.75,1};
  GLfloat specular[4] = { 0,0,0,1};
  GLfloat ambient[4] = { 0.2, 0.2, 0.2, 1.0 };

  GLfloat zero[4] = {0,0,0,0};
  glLightfv(GL_LIGHT1, GL_POSITION, position);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT1, GL_SPECULAR, specular);
  glLightfv(GL_LIGHT1, GL_AMBIENT, zero);
  glEnable(GL_LIGHT1);
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
  glEnable(GL_COLOR_MATERIAL);
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);

  GLfloat spec_mat[4] = {1,1,1,1};
  float glexponent = 30;
  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, &glexponent);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec_mat);

  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  float back_color[] = { 0.0,0.0,1.0,1};
  glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, back_color);
  glEnable(GL_LIGHT1);
}


void GLCanvas::display(void) {

  HandleGLError("start display");
  // Clear the display buffer, set it to the background color
  glClearColor(1,1,1,0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Set the camera parameters
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  InitLight(); // light will be a headlamp!
  camera->glPlaceCamera();

  glDisable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);
  
  glMatrixMode(GL_MODELVIEW);

  //assert (Smoke != NULL);
  assert (smoke != NULL);
  //bbox.Set(Smoke->getBoundingBox()); 
  bbox.Set(smoke->getBoundingBox());

  //// center the volume in the window
  //Matrix m;
  //m.setToIdentity();
  //Vec3f center;
  //bbox.getCenter(center);
  //m *= Matrix::MakeScale(1/double(bbox.maxDim()));
  //m *= Matrix::MakeTranslation(-center); 
  //float matrix_data[16];
  //m.glGet(matrix_data);
  //glMultMatrixf(matrix_data);
   mesh->drawVBOs();
   
  if(smoke) smoke->drawVBOs();
  if (args->bounding_box) {
    bbox.setupVBOs();
    bbox.drawVBOs();
  }
  
  glutSwapBuffers();
  HandleGLError("end display");
}

// ========================================================
// Callback function for window resize
// ========================================================

void GLCanvas::reshape(int w, int h) {
  args->width = w;
  args->height = h;

  // Set the OpenGL viewport to fill the entire window
  glViewport(0, 0, (GLsizei)args->width, (GLsizei)args->height);

  // Set the camera parameters to reflect the changes
  camera->glInit(args->width, args->height);
}

// ========================================================
// trace a ray through pixel (i,j) of the image an return the color
// ========================================================
Vec3f GLCanvas::TraceRay(double i, double j) {
  // compute and set the pixel color
  int max_d = my_max(args->width,args->height);

   Vec3f color;
   double x = (i+0.5-args->width/2.0)/double(max_d)+0.5;
   double y = (j+0.5-args->height/2.0)/double(max_d)+0.5;	
   Hit hit;	
   Ray r = camera->generateRay(x,y); 
   smoke->AddMainSegment(r,0,hit.getT());
   color = raytracer->TraceRay(r,hit,args->num_bounces);
   
   // add that ray for visualization
   smoke->AddMainSegment(r,0,hit.getT());
 
   // return the color
  return color;
}
// ========================================================
// Callback function for mouse click or release
// ========================================================

void GLCanvas::mouse(int button, int /*state*/, int x, int y) {
  // Save the current state of the mouse.  This will be
  // used by the 'motion' function
  mouseButton = button;
  mouseX = x;
  mouseY = y;

  shiftPressed = (glutGetModifiers() & GLUT_ACTIVE_SHIFT) != 0;
  controlPressed = (glutGetModifiers() & GLUT_ACTIVE_CTRL) != 0;
  altPressed = (glutGetModifiers() & GLUT_ACTIVE_ALT) != 0;
}

// ========================================================
// Callback function for mouse drag
// ========================================================

void GLCanvas::motion(int x, int y) {
  // Control or Shift or Alt pressed = zoom
  // (don't move the camera, just change the angle or image size)
  if (controlPressed || shiftPressed || altPressed) {
    camera->zoomCamera(mouseY-y);
  }
  // Left button = rotation
  // (rotate camera around the up and horizontal vectors)
  else if (mouseButton == GLUT_LEFT_BUTTON) {
    camera->rotateCamera(0.005*(mouseX-x), 0.005*(mouseY-y));
  }
  // Middle button = translation
  // (move camera perpendicular to the direction vector)
  else if (mouseButton == GLUT_MIDDLE_BUTTON) {
    camera->truckCamera(mouseX-x, y-mouseY);
  }
  // Right button = dolly or zoom
  // (move camera along the direction vector)
  else if (mouseButton == GLUT_RIGHT_BUTTON) {
    camera->dollyCamera(mouseY-y);
  }
  mouseX = x;
  mouseY = y;

  // Redraw the scene with the new camera parameters
  glutPostRedisplay();
}

// ========================================================
// Callback function for keyboard events
// ========================================================

void GLCanvas::keyboard(unsigned char key, int x, int y) {
  //args->raytracing_animation = false;
  switch (key) {
  case 'a': case 'A':
    // toggle continuous animation
    args->animate = !args->animate;
    if (args->animate) 
      printf ("animation started, press 'A' to stop\n");
    else
      printf ("animation stopped, press 'A' to start\n");
    break;
  case 'm':  case 'M': 
    args->particles = !args->particles;
    glutPostRedisplay();
    break; 
  case 'v':  case 'V': 
    args->velocity = !args->velocity;
    glutPostRedisplay();
    break; 
  case 'e':  case 'E':   // "faces"/"edges"
    args->face_velocity = !args->face_velocity;
    glutPostRedisplay();
    break; 
  case 'd':  case 'D': 
    args->dense_velocity = (args->dense_velocity+1)%4;
	if (smoke) smoke->setupVBOs();
    glutPostRedisplay();
    break; 
  case 's':  case 'S': 
    args->surface = !args->surface;
    glutPostRedisplay();
    break; 
  case 'w':  case 'W':
    args->wireframe = !args->wireframe;
    glutPostRedisplay();
    break;
  case 'b':  case 'B':
    args->bounding_box = !args->bounding_box;
    glutPostRedisplay();
    break;
  case 'c':  case 'C': 
    args->cubes = !args->cubes;
    glutPostRedisplay();
    break; 
  case 'p':  case 'P': 
    args->pressure = !args->pressure;
    glutPostRedisplay();
    break; 
  case 'x':  case 'X': 
    // reset system
    Load();
    glutPostRedisplay();
    break; 
  case 'r': case'R':
	   // animate raytracing of the scene
    args->raytracing_animation = !args->raytracing_animation;
    if (args->raytracing_animation) {
      raytracing_skip = my_max(args->width,args->height) / 10;
      if (raytracing_skip % 2 == 0) raytracing_skip++;
      assert (raytracing_skip >= 1);
      raytracing_x = raytracing_skip/2;
      raytracing_y = raytracing_skip/2;
      display(); // clear out any old rendering
      printf ("raytracing animation started, press 'R' to stop\n");
    } else
      printf ("raytracing animation stopped, press 'R' to start\n");    
    break;
	case 't':  case 'T': {
		// visualize the ray tree for the pixel at the current mouse position
		int i = x;
		int j = glutGet(GLUT_WINDOW_HEIGHT)-y;
		smoke->Activate();
		raytracing_skip = 1;
		TraceRay(i,j);
		smoke->Deactivate();
		// redraw
		mesh->setupVBOs();
	    smoke->setupVBOsR();
	
	    glutPostRedisplay();

		 break; 
	}

    break;

  case 'o': case 'O':
	  args->octree = !args->octree;
	  glutPostRedisplay();
	  break;
 // RADIOSITY STUFF
   case ' ': 
    // a single step of radiosity
    //raytracer->Iterate();
    mesh->setupVBOs();
    glutPostRedisplay();
    break;
  case '+': case '=':
    std::cout << "timestep doubled:  " << args->timestep << " -> ";
    args->timestep *= 2.0; 
    std::cout << args->timestep << std::endl;
	if (smoke) smoke->setupVBOs();
    glutPostRedisplay();
    break;
  case '-': case '_':
    std::cout << "timestep halved:  " << args->timestep << " -> ";
    args->timestep /= 2.0; 
    std::cout << args->timestep << std::endl;

	if (smoke) smoke->setupVBOs();
    glutPostRedisplay();
    break;
  case 'q':  case 'Q':
    delete GLCanvas::raytracer;
	delete smoke;
	smoke = NULL;
    delete camera;
    camera = NULL;
	 delete GLCanvas::raytracer;
    delete GLCanvas::mesh;
    printf ("program exiting\n");
    exit(0);
    break;
  default:
    printf("UNKNOWN KEYBOARD INPUT  '%c'\n", key);
  }
}


void GLCanvas::idle() {
  if (args->animate) {
    // do 10 steps of animation before rendering
    for (int i = 0; i < 10; i++) {
  
	  if (smoke) smoke->Animate();
    }
	mesh->setupVBOs();
    glutPostRedisplay();
  }
  if (args->raytracing_animation) {

	  // add contributions from each light that is not in shadow
	int num_lights = mesh->getLights().size();
	float b =.4; //scattering coefficient
	float a = .2; //absorption coefficient
	float c = b+a;
	for (int i = 0; i < num_lights; i++) 
	{
		Face *f = mesh->getLights()[i];
		Vec3f lightColor = f->getMaterial()->getEmittedColor() * f->getArea();
		Vec3f lightCentroid = f->computeCentroid();
		smoke->oc->calculateTransmittanceOfBB(lightCentroid,c,lightColor);
	}
    // draw 100 pixels and then refresh the screen and handle any user input
    glDisable(GL_LIGHTING);
    glDrawBuffer(GL_FRONT);
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glPointSize(raytracing_skip);
    glBegin(GL_POINTS);
    for (int i = 0; i < 100; i++) {
      if (!DrawPixel()) {
	args->raytracing_animation = false;
	break;
      }
    }
    glEnd();
    glFlush();
  }
}


// ========================================================
// ========================================================

int HandleGLError(const std::string &message) {
  GLenum error;
  int i = 0;
  while ((error = glGetError()) != GL_NO_ERROR) {
    if (message != "") {
      std::cout << "[" << message << "] ";
    }
    std::cout << "GL ERROR(" << i << ") " << gluErrorString(error) << std::endl;
    i++;
  }
  if (i == 0) return 1;
  return 0;
}

// ========================================================
// ========================================================



// Scan through the image from the lower left corner across each row
// and then up to the top right.  Initially the image is sampled very
// coarsely.  Increment the static variables that track the progress
// through the scans
int GLCanvas::DrawPixel() {

	if (raytracing_x > args->width) {
	raytracing_x = raytracing_skip/2;
	raytracing_y += raytracing_skip;
	}
	if (raytracing_y > args->height) {
	if (raytracing_skip == 1) return 0;
	raytracing_skip = raytracing_skip / 2;
	if (raytracing_skip % 2 == 0) raytracing_skip++;
	assert (raytracing_skip >= 1);
	raytracing_x = raytracing_skip/2;
	raytracing_y = raytracing_skip/2;
	glEnd();
	glPointSize(raytracing_skip);
	glBegin(GL_POINTS);
	}

	// compute the color and position of intersection
	Vec3f color =  TraceRay(raytracing_x, raytracing_y);
	double r = linear_to_srgb(color.x());
	double g = linear_to_srgb(color.y());
	double b = linear_to_srgb(color.z());
	//std::cout<<r<<" "<<g<<" "<<b<<std::endl;
	glColor3f(r,g,b);
	 
	double x = 2 * (raytracing_x/double(args->width)) - 1;
	double y = 2 * (raytracing_y/double(args->height)) - 1;
	glVertex3f(x,y,-1);
  
	raytracing_x += raytracing_skip;

	return 1;
	
}

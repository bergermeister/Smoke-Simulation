#ifndef __ARG_PARSER_H__
#define __ARG_PARSER_H__

#include <cassert>
#include <string>

#include "vectors.h"
#include "MersenneTwister.h"

#define NUM_RENDER_MODES 6
enum RENDER_MODE { RENDER_MATERIALS, RENDER_RADIANCE, RENDER_FORM_FACTORS, 
		   RENDER_LIGHTS, RENDER_UNDISTRIBUTED, RENDER_ABSORBED };

// ================================================================================
// ================================================================================

class ArgParser {

public:

  ArgParser() { DefaultValues(); }

  ArgParser(int argc, char *argv[]) {
    DefaultValues();

    for (int i = 1; i < argc; i++) {
	  if (argv[i] == std::string("-smoke")){
		  i++; assert(i < argc);
		  smoke_file = argv[i];
      } 
	  else if (argv[i] == std::string("-mesh")){
		  i++; assert(i < argc);
		  mesh_file = argv[i];}
	  else if (argv[i] == std::string("-size")) {
        i++; assert (i < argc); 
		width = height = atoi(argv[i]);
      } else if (argv[i] == std::string("-timestep")) {
		i++; assert (i < argc); 
		timestep = atof(argv[i]);
        assert (timestep > 0);
      } else if (!strcmp(argv[i],"-num_bounces")) {
	i++; assert (i < argc); 
	num_bounces = atoi(argv[i]);
      } else if (!strcmp(argv[i],"-num_shadow_samples")) {
	i++; assert (i < argc); 
	num_shadow_samples = atoi(argv[i]);
      } else {
		printf ("whoops error with command line argument %d: '%s'\n",i,argv[i]);
		assert(0);
      }
    }
  }

  // ===================================
  // ===================================

  void DefaultValues() {
    width = 500;
    height = 500;

    timestep = 0.01;
    animate = false;

    particles = true;
    velocity = true;

    face_velocity = 0;
    dense_velocity = 0;

    surface = false;
    isosurface = 0.7;

    wireframe = false;
    bounding_box = true;
	octree = false;
    cubes = false;
    pressure = false;

    gravity = Vec3f(0,-9.8,0);
	
	 // RADIOSITY PARAMETERS
    render_mode = RENDER_MATERIALS;
    wireframe = false;
    sphere_horiz = 8;
    sphere_vert = 6;

	// RAYTRACING PARAMETERS
	 intersect_backfacing = false;
	 raytracing_animation = false;
	 num_bounces = 0;
	 ambient_light = Vec3f(0.1,0.1,0.1);
	 num_shadow_samples = 0;
    // uncomment for deterministic randomness
    // mtrand = MTRand(37);
    
  }

  // ===================================
  // ===================================
  // REPRESENTATION
  // all public! (no accessors)

   // BASIC RENDERING PARAMETERS
  char *input_file;
  int width;
  int height;

  std::string mesh_file;
  std::string smoke_file;

  // animation control
  double timestep;
  bool animate;
  Vec3f gravity;

   // RADIOSITY PARAMETERS
  enum RENDER_MODE render_mode;
  bool wireframe;
  int sphere_horiz;
  int sphere_vert;

  // RAYTRACING PARAMETERS
  bool raytracing_animation;
  int num_bounces;
  int num_shadow_samples;
  Vec3f ambient_light;
  bool intersect_backfacing;

  // display option toggles 
  // (used by both)
  bool particles;
  bool velocity;
  bool surface;
  bool bounding_box;
  bool octree;

  // used by Smoke
  int face_velocity;
  int dense_velocity;
  double isosurface;
  bool cubes;
  bool pressure;

  // default initialization
  MTRand mtrand;

};

// ================================================================================

#endif

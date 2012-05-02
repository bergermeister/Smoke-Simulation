#ifndef _RAY_TRACER_
#define _RAY_TRACER_

#include <vector>
#include<map>
#include "ray.h"
#include "hit.h"
#include "smoke.h"
#include "face.h"

class Mesh;
class ArgParser;
class Radiosity;
class PhotonMapping;

// ====================================================================
// ====================================================================
// This class manages the ray casting and ray tracing work.

class RayTracer {

public:

  // CONSTRUCTOR & DESTRUCTOR
  RayTracer(Smoke *s, ArgParser *a,Mesh *m) {
    smoke = s;
    args = a;
	mesh = m;
	num_form_factor_samples = 10;
  }  

  // casts a single ray through the scene geometry and finds the closest hit
  bool CastRay(const Ray &ray, Hit &h, bool use_sphere_patches) const;
  bool ParticleInGrid(const Vec3f position,const BoundingBox *b) const;
  bool ParticleInCircle(const Vec3f pos, const Vec3f center, float radius,float width) const;

  // does the recursive work
  Vec3f TraceRay(Ray &ray, Hit &hit, int bounce_count = 0) const;
  static Radiosity *radiosity;
private:

 Vec3f Trace(const Ray &ray, Face *f, Vec3f end) const;
 Vec3f Scattering(const Ray &ray,std::vector<SmokeParticle*>pr,int numParticles,Vec3f from,float radius,float width,float T) const;

  // REPRESENTATION
  Smoke *smoke;
  ArgParser *args;
  Mesh *mesh;


  //RADIOSITY
   // length n vectors
  double *area;
  Vec3f *undistributed; // energy per unit area
  Vec3f *absorbed;      // energy per unit area
  Vec3f *radiance;      // energy per unit area

  int max_undistributed_patch;  // the patch with the most undistributed energy
  double total_undistributed;    // the total amount of undistributed light
  double total_area;             // the total area of the scene

   // a nxn matrix F_i,j radiant energy leaving i arriving at j
  double *formfactors;
  int num_form_factor_samples;
  void ComputeFormFactors() const;
  void Iterate() const; 

  void setArea(int i, double value);
  void setUndistributed(int i, Vec3f value);
  void findMaxUndistributed();
  void setAbsorbed(int i, Vec3f value);
  void setRadiance(int i, Vec3f value);

};

// ====================================================================
// ====================================================================

#endif

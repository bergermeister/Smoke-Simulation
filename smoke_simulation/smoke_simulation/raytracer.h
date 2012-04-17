#ifndef _RAY_TRACER_
#define _RAY_TRACER_

#include <vector>
#include "ray.h"
#include "hit.h"
#include "smoke.h"
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
  RayTracer(Smoke *m, ArgParser *a) {
    smoke = m;
    args = a;
  }  

  // casts a single ray through the scene geometry and finds the closest hit
  bool CastRay(const Ray &ray, Hit &h, bool use_sphere_patches,BoundingBox *box,BoundingBox *grid) const;
  bool ParticleInGrid(const Vec3f position,const BoundingBox *b) const;
 

  void trace();
  // does the recursive work
  Vec3f TraceRay(Ray &ray, Hit &hit, int bounce_count = 0) const;

private:

  // REPRESENTATION
  Smoke *smoke;
  ArgParser *args;
};

// ====================================================================
// ====================================================================

#endif

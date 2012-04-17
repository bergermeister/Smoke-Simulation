#ifndef _MATERIAL_H_
#define _MATERIAL_H_

#include "glCanvas.h"
#include <cassert>
#include <string>
#include "vectors.h"

class ArgParser;
class Ray;
class Hit;

// ====================================================================
// ====================================================================
// A simple Phong-like material 

class Material {

public:

  Material(const Vec3f &d_color,const Vec3f &r_color, const Vec3f &e_color, double roughness_)
  {
  
    diffuseColor = d_color;   
    reflectiveColor = r_color;
    emittedColor = e_color;
    roughness = roughness_;
    // need to initialize texture_id after glut has started
  }
  
  ~Material();

  // ACCESSORS
  const Vec3f& getDiffuseColor() const { return diffuseColor; }
  const Vec3f getDiffuseColor(double s, double t) const;
  const Vec3f& getReflectiveColor() const { return reflectiveColor; }
  const Vec3f& getEmittedColor() const { return emittedColor; }  
  double getRoughness() const { return roughness; } 


  // SHADE
  // compute the contribution to local illumination at this point for
  // a particular light source
  Vec3f Shade
  (const Ray &ray, const Hit &hit, const Vec3f &dirToLight, 
   const Vec3f &lightColor, ArgParser *args) const;
  
protected:

  Material() { exit(0); }
  Material(const Material&) { exit(0); }
  const Material& operator=(const Material&) { exit(0); }


  // REPRESENTATION
  Vec3f diffuseColor;
  Vec3f reflectiveColor;
  Vec3f emittedColor;
  double roughness;

};

// ====================================================================
// ====================================================================

#endif


  

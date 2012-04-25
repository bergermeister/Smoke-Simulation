#include "material.h"
#include "utils.h"
#include "ray.h"
#include "hit.h"
#include <math.h>
// ==================================================================
// DESTRUCTOR
// ==================================================================
Material::~Material()
{
}

// ==================================================================
// TEXTURE LOOKUP FOR DIFFUSE COLOR
// ==================================================================
const Vec3f Material::getDiffuseColor(double s, double t) const {
 return diffuseColor;
}


// ==================================================================
// PHONG LOCAL ILLUMINATION

// this function should be called to compute the light contributed by
// a particular light source to the intersection point.  Note that
// this function does not calculate any global effects (e.g., shadows). 

Vec3f Material::Shade(const Ray &ray, const Hit &hit, const Vec3f &dirToLight, const Vec3f &lightColor, ArgParser *args) const {
  
  Vec3f point = ray.pointAtParameter(hit.getT());
  Vec3f n = hit.getNormal();
  Vec3f e = ray.getDirection()*-1.0f;
  Vec3f l = dirToLight;
  
  Vec3f answer = Vec3f(0,0,0);

  // emitted component
  // -----------------
  answer += getEmittedColor();

  // diffuse component
  // -----------------
  double dot_nl = n.Dot3(l);
  if (dot_nl < 0) dot_nl = 0;
  answer += lightColor * getDiffuseColor(hit.get_s(),hit.get_t()) * dot_nl;

  // specular component (Phong)
  // ------------------
  // make up reasonable values for other Phong parameters
  Vec3f specularColor = reflectiveColor;
  double exponent = 100;

  // compute ideal reflection angle
  Vec3f r = (l*-1.0f) + n * (2 * dot_nl);
  r.Normalize();
  double dot_er = e.Dot3(r);
  if (dot_er < 0) dot_er = 0;
  answer += lightColor*specularColor*pow(dot_er,exponent)* dot_nl;

  return answer;
}

float AlternateFog( float cosVal, float distToFrag )
{
	float fex = exp( -distToFrag*(0.15+0.15) );
	float ins = (1-fex);
	float brt = 3.0/(16.0*3.14159)*0.15*(1.0+cosVal*cosVal);
	float bmt = 1.0/(4.0*3.14159)*0.15*(1.0-0.95)*(1.0-0.95)/(pow((1.0+0.95*0.95-2*0.95*cosVal),1.5) );
	float lint = (brt+bmt)/(0.15+0.15)*ins;
	return lint;
}


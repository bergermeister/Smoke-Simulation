#ifndef _UTILS_H
#define _UTILS_H

#include <cassert>

// ======================================================================

#define square(x) ((x)*(x))
// helper for VBOs
#define BUFFER_OFFSET(i) ((char *)NULL + (i))

#if defined(_WIN32) 
// windows already has them defined...
#define my_max max
#define my_min min
#else
#define my_max std::max
#define my_min std::min
#endif


// ======================================================================

inline Vec3f computeNormal(const Vec3f &p1, const Vec3f &p2, const Vec3f &p3) {
  Vec3f v12 = p2;
  v12 -= p1;
  Vec3f v23 = p3;
  v23 -= p2;
  Vec3f normal;
  Vec3f::Cross3(normal,v12,v23);
  normal.Normalize();
  return normal;
}


inline double triInterpolate(double x_frac, double y_frac, double z_frac,
			     double a, double b, double c, double d, double e, double f, double g, double h) {
  
  assert (x_frac >= 0 && x_frac <= 1);
  assert (y_frac >= 0 && y_frac <= 1);
  assert (z_frac >= 0 && z_frac <= 1);

  // trilinear interpolation
  double ab = (1-z_frac)*a + z_frac*b;
  double cd = (1-z_frac)*c + z_frac*d;
  double ef = (1-z_frac)*e + z_frac*f;
  double gh = (1-z_frac)*g + z_frac*h;
  double abcd = (1-y_frac)*ab + y_frac*cd;
  double efgh = (1-y_frac)*ef + y_frac*gh;
  double abcdefgh = (1-x_frac)*abcd + x_frac*efgh;
  return abcdefgh;
}


inline double AreaOfTriangle(double a, double b, double c) {
  // Area of Triangle =  (using Heron's Formula)
  //  sqrt[s*(s-a)*(s-b)*(s-c)]
  //    where s = (a+b+c)/2
  // also... Area of Triangle = 0.5 * x * c
  double s = (a+b+c) / (double)2;
  double tmp = s*(s-a)*(s-b)*(s-c);
  if (tmp < 0) return 0;
  double answer = sqrt(tmp);
  return answer;
}


inline double AreaOfTriangle(const Vec3f &a, const Vec3f &b, const Vec3f &c) {
  return AreaOfTriangle((a-b).Length(), (b-c).Length(), (c-a).Length());
}

// ======================================================================

#endif


#ifndef _BOUNDING_BOX_H_
#define _BOUNDING_BOX_H_

#include "glCanvas.h"
#include <cassert>
#include <vector>
#include <algorithm>
#include "vectors.h"
#include "utils.h"
#include "vbo_structs.h"

// ====================================================================

class SmokeParticle {
public:
	// CONSTRUCTORS
	SmokeParticle () { position = Vec3f(0,0,0); temperature = 0; density = 0; radius = 30; }
	SmokeParticle (Vec3f pos, double temp, double dens)	{ position = pos; temperature = temp; density = dens; radius = 10; }
	// accessor
	Vec3f getPosition() const { return position; }
	double getTemperature() const { return temperature; }
	double getRadius() const { return radius; }
	double getDensity() const { return density; }
	// modifer
	void setPosition(Vec3f p) { position = p; }
private:
	// representation
	Vec3f position;
	double temperature;
	double density;
	double radius;
};

// ====================================================================

enum CELL_STATUS { CELL_EMPTY, CELL_SURFACE, CELL_FULL };
class BoundingBox {

public:

  // ========================
  // CONSTRUCTOR & DESTRUCTOR
  BoundingBox() 
  { 
	  Set(Vec3f(0,0,0),Vec3f(0,0,0)); 
      pressure = 0;
	  status = CELL_SURFACE; 
	  u_plus = 0;
	  v_plus = 0;
	  w_plus = 0;
	  new_u_plus = 0;
	  new_v_plus = 0;
	  new_w_plus = 0;
  
  }
  BoundingBox(const Vec3f &v) { Set(v,v); }
  BoundingBox(const Vec3f &_minimum, const Vec3f &_maximum) { Set(_minimum,_maximum); }
  ~BoundingBox() 
  {
	 for (unsigned int i = 0; i < particles.size(); i++) 
          delete particles[i];
  }


  // =========
  // ACCESSORS
  void Get(Vec3f &_minimum, Vec3f &_maximum) const {
    _minimum = minimum;
    _maximum = maximum; }
  Vec3f getMin() const { return minimum; }
  Vec3f getMax() const { return maximum; }
  void getCenter(Vec3f &c) const {
    c = maximum; 
    c -= minimum;
    c *= 0.5f;
    c += minimum;
  }
  double maxDim() const {
    double x = maximum.x() - minimum.x();
    double y = maximum.y() - minimum.y();
    double z = maximum.z() - minimum.z();
    return my_max(x,my_max(y,z));
  }

   // =========
  // ACCESSORS "CELL"
  double getPressure() const { return pressure; }
  enum CELL_STATUS getStatus() const { return status; }
  double get_u_plus() const { return u_plus; }
  double get_v_plus() const { return v_plus; }
  double get_w_plus() const { return w_plus; }
  double get_new_u_plus() const { return new_u_plus; }
  double get_new_v_plus() const { return new_v_plus; }
  double get_new_w_plus() const { return new_w_plus; }
  int numParticles() const { return particles.size(); }
  std::vector<SmokeParticle*>& getParticles() { return particles; }

  // =========
  // MODIFIERS "CELL"
  void setPressure(double p) { pressure = p; }
  void setStatus(enum CELL_STATUS s) { status = s; }
  void set_u_plus(double f) { u_plus = new_u_plus = f; }
  void set_v_plus(double f) { v_plus = new_v_plus = f; }
  void set_w_plus(double f) { w_plus = new_w_plus = f; }
  void set_new_u_plus(double f) { new_u_plus = f; }
  void set_new_v_plus(double f) { new_v_plus = f; }
  void set_new_w_plus(double f) { new_w_plus = f; }
  void adjust_new_u_plus(double f) { new_u_plus += f; }
  void adjust_new_v_plus(double f) { new_v_plus += f; }
  void adjust_new_w_plus(double f) { new_w_plus += f; }
  void copyVelocity() { 
    u_plus = new_u_plus; new_u_plus = 0;
    v_plus = new_v_plus; new_v_plus = 0;
    w_plus = new_w_plus; new_w_plus = 0; 
  }
  void addParticle(SmokeParticle *p) {
    assert(p != NULL);
    particles.push_back(p); 
  }
  void removeParticle(SmokeParticle *p) {
    assert(p != NULL);
    for (std::vector<SmokeParticle*>::iterator i = particles.begin(); i != particles.end(); i++) 
	{
      if (*i == p)
	  {
        particles.erase(i);
        return;
      }
    }
    assert (0); 
  }

  // =========
  // MODIFIERS
  void Set(const BoundingBox &bb) {
      Set(bb.minimum,bb.maximum); 
  }
  void Set(const Vec3f &_minimum, const Vec3f &_maximum) {
    assert (minimum.x() <= maximum.x() &&
	    minimum.y() <= maximum.y() &&
	    minimum.z() <= maximum.z());
    minimum = _minimum;
    maximum = _maximum; }
  void Extend(const Vec3f &v) {
    minimum = Vec3f(my_min(minimum.x(),v.x()),
		    my_min(minimum.y(),v.y()),
		    my_min(minimum.z(),v.z()));
    maximum = Vec3f(my_max(maximum.x(),v.x()),
		    my_max(maximum.y(),v.y()),
		    my_max(maximum.z(),v.z())); 
  }  
  void Extend(const BoundingBox &bb) {
    Extend(bb.minimum);
    Extend(bb.maximum); 
  }

  // =========
  // DEBUGGING 
  void Print(const char *s="") const {
    printf ("BOUNDING BOX %s: %f %f %f  -> %f %f %f\n", s,
            minimum.x(),minimum.y(),minimum.z(),
            maximum.x(),maximum.y(),maximum.z()); }

  void initializeVBOs();
  void setupVBOs();
  void drawVBOs();
  void cleanupVBOs();

private:

  // ==============
  // REPRESENTATION
  Vec3f minimum;
  Vec3f maximum;
  
  GLuint bb_verts_VBO;
  GLuint bb_edge_indices_VBO;

  double pressure;
  enum CELL_STATUS status;

  // velocities at the center of each face (flowing in the positive direction)
  double u_plus,v_plus,w_plus;
  double new_u_plus,new_v_plus,new_w_plus;

  std::vector<SmokeParticle*> particles;
};

// ====================================================================
// ====================================================================

#endif


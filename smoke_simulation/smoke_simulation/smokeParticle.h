#ifndef SMOKE_PARTICLE_H
#define SMOKE_PARTICLE_H


#include "glCanvas.h"

class Ray;
class Hit;
class Material;


class SmokeParticle {
public:
	// CONSTRUCTORS
	SmokeParticle () { position = Vec3f(0,0,0); temperature = 0; density = 0; radius = .2; setMaterial(); }
	SmokeParticle (Vec3f pos, double temp, double dens)	{ position = pos; temperature = temp; density = dens; radius = .2; setMaterial(); }
	
	// accessor 
    Material* getMaterial() const { return material; }
	Vec3f getPosition() const { return position; }
	double getTemperature() const { return temperature; }
	double getRadius() const { return radius; }
	Vec3f getCenter() const {return center;}
	double getDensity() const { return density; }

	// modifer
	void setPosition(Vec3f p) 
	{ 
		position = p;
		center = p;
	}

   // for ray tracing
   bool intersect(const Ray &r, Hit &h);
   float DistanceEstimator(Vec3f p);
private:
	// representation
	Vec3f position;
	double temperature;
	double density;
	double radius;
	Vec3f center;
	Material *material;

	void setMaterial();
	
};

// ====================================================================
#endif
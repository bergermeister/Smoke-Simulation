#ifndef PARTICLE_H
#define PARTICLE_H

#include "glCanvas.h"

class particle {
public:
	particle(float p[3],float v[3],float l,float m,float a);
	void update();
	void draw();
	bool isDead();
	double radius;  

private:
	float vel[3];   //elocity
	float pos[3];   //position
	float life;     //life spam
	float mass;
	float alpha;
	bool dead;

	bool isColliding();
	void handleCollisions();
};

#endif
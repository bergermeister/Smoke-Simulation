#ifndef _PARTICLESYSTEM_H_
#define _PARTICLESYSTEM_H_

#include "particle.h"
#include <vector>

class particleSystem {
public:
	particleSystem();
	void update();
	void render();
	void draw();

	int getNumParticles();
	void addParticle(particle p);

private:
	int numParticles;
	std::vector<particle> particles;

};

#endif
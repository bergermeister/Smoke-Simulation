#include "particleSystem.h"


//-------------------------------------------------------------------------------
//Constructor
particleSystem::particleSystem()
{

}

//-------------------------------------------------------------------------------
//Function: Update P.S
void particleSystem::update()
{
	int size = particles.size();
	int i =0;
	while(i<size)
	{
		particles[i].update();
		if(particles[i].isDead())
		{
			particles.erase(particles.begin()+i);  //if dead take out
		}
		else {i++;}

	}
}

//-------------------------------------------------------------------------------
//Function: Draw P.S
void particleSystem::draw()
{
	for(int i =0;i<particles.size();i++)
	{
		particles[i].draw();
	}
}

//-------------------------------------------------------------------------------
//Function: Add particle to P.S
void particleSystem::addParticle(particle p )
{
	particles.push_back(p);
}
//-------------------------------------------------------------------------------
//Function: Return size vector particles
int particleSystem::getNumParticles()
{
	return particles.size();
}
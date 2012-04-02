#include "particle.h"

static const float MOMENTUM = 0.5f;
static const float FLUID_FORCE = 0.6f;

//-------------------------------------------------------------------------------
//Constructor
particle::particle(float p[3],float v[3],float l,float m,float a)
{
	pos[0] = p[0];
	pos[1] = p[1];
	pos[2] = p[2];
	vel[0] = v[0];
	vel[1] = v[1];
	vel[2] = v[2];
	life = l;
	alpha = a;
	dead = false;
}

//-------------------------------------------------------------------------------
//Function: Update
void particle::update()
{
	life --;
	if(life <0)
		dead = true;
	// fade out a bit (and kill if alpha == 0);
	alpha *= 0.999f;
	if( alpha < 0.01f )
		alpha = 0;

	if( alpha == 0 )
	   dead = true;


	/* Update Velocity */

	/* Update Pos */

	/* Check if colliding */
	bool c = isColliding();
	if(c)
		handleCollisions();
}

//-------------------------------------------------------------------------------
//Function: Check if particle colliding with some obj
bool particle::isColliding()
{
	bool pass = false;

	return pass;
}

//-------------------------------------------------------------------------------
//Function: Handles collision of particle if colliding
void particle::handleCollisions()
{

}
//-------------------------------------------------------------------------------
//Function: Returns is particle is dead or not to remove of vector in P.S
bool particle::isDead()
{
	return dead;
}

//-------------------------------------------------------------------------------
//Function: Draw particle
void particle::draw()
{
	glEnable(GL_BLEND);
		glDisable( GL_TEXTURE_2D );
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glColor4f(2,2,2,alpha);
		glTranslatef(pos[0],pos[1],pos[2]);
		glutSolidSphere(radius,10, 100);
		//glutWireSphere(GLdouble radius,GLint slices, GLint stacks);
	glDisable(GL_BLEND);

}
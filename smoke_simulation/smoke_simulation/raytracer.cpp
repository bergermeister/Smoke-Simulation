#include "raytracer.h"
//#include "material.h"
#include "vectors.h"
#include "argparser.h"
#include "smoke.h"
#include "utils.h"

std::vector<SmokeParticle*>particles;
float Volume;

// ===========================================================================
// casts a single ray through the scene geometry and finds the closest hit
bool RayTracer::CastRay(const Ray &ray, Hit &h, bool use_rasterized_patches,BoundingBox *box,BoundingBox *grid) const {
	
	float MinimumDistance = 0.4;
	bool answer  = false;
	bool hit     = false;
	bool out     = false; //out of main grid
    bool in      = false;
	int steps=0;  //total num, of steps taken

	Vec3f direction = ray.getDirection();
	Vec3f from = ray.getOrigin();

	//Create the bounding box that surrounds our path space
	Vec3f max = Vec3f((float)from.x()+ MinimumDistance, (float)from.y()+ MinimumDistance,(float)from.z()+ MinimumDistance);
	Vec3f min = Vec3f((float)from.x()- MinimumDistance, (float)from.y()- MinimumDistance,(float)from.z()- MinimumDistance);
	BoundingBox *bbBox=new BoundingBox();
	bbBox->Set(min,max);
	BoundingBox *mBox =smoke->oc->getCell(from.x(),from.y(),from.z());
	//mBox->Set(min,max);
	std::vector<SmokeParticle *> pp = mBox->getParticles();
	//smoke->oc->CollectParticlesInBox(*mBox,pp);

	while(!out)
	{

		steps++;
		if(hit)
		{
			hit=false;
			from+=direction;
		}
	
		/* if in marching path box */
		if(ParticleInGrid(from,mBox))
		{
			
			hit=true;
			std::cout<<"in"<<std::endl;
			for(int i=0;i<pp.size();i++)
			{
				in = true;
				float distance =pp[i]->DistanceEstimator(from);
				if(ParticleInGrid(pp[i]->getPosition(),bbBox))
				{
					answer = true;
					std::cout<<"yes"<<std::endl;
					//Vec3f n = ray.pointAtParameter(from.Length());
					//n.Normalize();
					particles.push_back(pp[i]);
					//h.set(from.Length(), pp[i]->getMaterial(), n);
					
					/*red particles*/
					smoke->smoke_particlesHit.push_back(VBOPos(pp[i]->getPosition()));
					glBindBuffer(GL_ARRAY_BUFFER,smoke->smoke_particles_Hit_VBO); 
					glBufferData(GL_ARRAY_BUFFER,sizeof(VBOPos)*smoke->smoke_particlesHit.size(),&smoke->smoke_particlesHit[0],GL_STATIC_DRAW); 
				}
			}
			if(in)
			{
				in = false;
				Vec3f Min = bbBox->getMin();
				Vec3f Max = bbBox->getMax();
				float width= max.x() - min.x();
				float height = max.y() - min.y();
				float depth = min.z() - max.z();
				Volume +=width*height*depth;
			}
		}
		else
		{
			smoke->smoke_particlesHit.push_back(VBOPos(from));
			glBindBuffer(GL_ARRAY_BUFFER,smoke->smoke_particles_Hit_VBO); 
			glBufferData(GL_ARRAY_BUFFER,sizeof(VBOPos)*smoke->smoke_particlesHit.size(),&smoke->smoke_particlesHit[0],GL_STATIC_DRAW);
			
			from+=direction;
			//Create the bounding box that surrounds the sphere
			max = Vec3f((float)from.x()+ MinimumDistance,(float) from.y()+ MinimumDistance,(float)from.z()+ MinimumDistance);
			min = Vec3f((float)from.x()- MinimumDistance, (float)from.y()- MinimumDistance,(float)from.z()- MinimumDistance);
		
			bbBox->Set(min,max);
			pp.clear();

			mBox = smoke->oc->getCell(from.x(),from.y(),from.z());
			pp = mBox->getParticles();
			std::cout<<"no"<<std::endl;
		}
		if(!ParticleInGrid(from,grid))
			out = true;

		smoke->smoke_particlesHit.push_back(VBOPos(from));
		glBindBuffer(GL_ARRAY_BUFFER,smoke->smoke_particles_Hit_VBO); 
		glBufferData(GL_ARRAY_BUFFER,sizeof(VBOPos)*smoke->smoke_particlesHit.size(),&smoke->smoke_particlesHit[0],GL_STATIC_DRAW); 
	}


	return answer;
}

bool RayTracer::ParticleInGrid(const Vec3f position,const BoundingBox *b) const
{
	if (position.x() > b->getMin().x()&&
		  position.y() > b->getMin().y() &&
		  position.z() > b->getMin().z()&&
		  position.x() < b->getMax().x()  &&
		  position.y() < b->getMax().y() &&
		  position.z() < b->getMax().z() )
		return true;
  return false;
}

// ===========================================================================
// does the recursive (shadow rays & recursive rays) work
Vec3f RayTracer::TraceRay(Ray &ray1, Hit &hit, int bounce_count) const {

	glDeleteBuffers(1, &smoke->smoke_particles_Hit_VBO);
	smoke->smoke_particlesHit.clear();

	Vec3f from = ray1.getOrigin();
	Vec3f n = ray1.getDirection();
	
	bool in=false;

	//while not in bbox
	int count=0;
	BoundingBox *grid = new BoundingBox();
	grid->Set(smoke->getBoundingBox());
	
	for(int i=0;i<25;i++)
	{
		from += n;
	}
	n.set(n.x()/20,n.y()/20,n.z()/20);
	do
	{
		in = ParticleInGrid(from,grid);	
		from += n;
		count++;
		smoke->smoke_particlesHit.push_back(VBOPos(from));
		glBindBuffer(GL_ARRAY_BUFFER,smoke->smoke_particles_Hit_VBO); 
		glBufferData(GL_ARRAY_BUFFER,sizeof(VBOPos)*smoke->smoke_particlesHit.size(),&smoke->smoke_particlesHit[0],GL_STATIC_DRAW); 
		if(count ==500)
		{
			std::cout<<"no11"<<std::endl;
			return Vec3f(srgb_to_linear(smoke->background_color.r()),srgb_to_linear(smoke->background_color.g()),srgb_to_linear(smoke->background_color.b()));
		}
	}while(!in);

	BoundingBox *box = smoke->oc->getCell(from.x(),from.y(),from.z());

	// First cast a ray and see if we hit anything.
	hit = Hit();
	Ray ray = Ray(from,n);
	bool intersect = CastRay(ray,hit,false,box,grid);
    
	// if there is no intersection, simply return the background color
	if (intersect == false) 
		return Vec3f(srgb_to_linear(smoke->background_color.r()),srgb_to_linear(smoke->background_color.g()),srgb_to_linear(smoke->background_color.b()));
	
	float density = particles.size()/Volume;

	Vec3f color;
	for(int i=0;i<particles.size();i++)
	{
		Material *m = particles[i]->getMaterial();
		Vec3f normal = particles[i]->getPosition();
		normal.Normalize();
		Vec3f point = ray.pointAtParameter(particles[i]->getPosition().Length());

		color = m->getEmittedColor();
	  // ----------------------------------------------
	  //  start with the indirect light (ambient light)
	  // Vec3f diffuse_color = m->getDiffuseColor();//hit.get_s(),hit.get_t());
	   // the usual ray tracing hack for indirect light
	   //color += diffuse_color * args->ambient_light;     

  // ----------------------------------------------
  // add contributions from each light that is not in shadow
  // int num_lights = box->getLights().size();
 
 //  for (int i = 0; i < num_lights; i++) {
	////soft shadow logic
	//Vec3f answerT;
 //   
	//Face *f = box->getLights()[i];
 //   Vec3f lightColor = f->getMaterial()->getEmittedColor() * f->getArea();
 //   Vec3f myLightColor;
 //   Vec3f lightCentroid = f->computeCentroid();
 //   Vec3f dirToLightCentroid = lightCentroid-point;
 //   dirToLightCentroid.Normalize();
 //   

 //   double distToLightCentroid = (lightCentroid-point).Length();
	//lightColor /=(M_PI*distToLightCentroid*distToLightCentroid);
 //   myLightColor = lightColor;// / (M_PI*distToLightCentroid*distToLightCentroid);

    // ===========================================
    // ASSIGNMENT:  ADD SHADOW & SOFT SHADOW LOGIC
    // ===========================================

	/* run once shooting to middle of light*/
		//if(args->num_shadow_samples == 1)
		//{
		//	Ray shadowR (point,dirToLightCentroid);
		//	Hit shadowH = Hit();
		//	CastRay(shadowR,shadowH,false);

		//	//IF no point bw point and light :
		//	  // add the lighting contribution from this particular light at this point
		//	
		//	if (shadowH.getMaterial()->getEmittedColor().Length() > 0.001)   // Not in shadow (collides with light source quad)
		//	{
		//		answer += m->Shade(ray,hit,dirToLightCentroid,myLightColor,args); 
		//	}
		//	// add the lighting contribution from this particular light at this point
		//	// (fix this to check for blockers between the light & this surface)
		//	smoke->AddShadowSegment(shadowR,0,shadowH.getT());	
	 //  } 
		//else if(args->num_shadow_samples >1)
	 //  {
		//   
		//  /* multiple shadow rays for SOFT SHADOW */
		//  for(int j =0;j<args->num_shadow_samples;j++)
		//  {
		//	  Vec3f randomP = f->RandomPoint();   //getting random point in light
		//	  Vec3f dirToLight = randomP - point;  //dir from random point
		//	  double dist = dirToLight.Length();
		//	  dirToLight.Normalize();
		//	  lightColor = f->getMaterial()->getEmittedColor() * f->getArea();

		//	  lightColor /= M_PI*dist*dist;

		//	  Ray shadowR (point ,dirToLight);
		//	  Hit shadowH = Hit();
		//	  CastRay(shadowR,shadowH,false);
		//	   // if (!CastRay(shadowR,shadowH,false))
		//	  if (shadowH.getMaterial()->getEmittedColor().Length() > 0.001)// Not in shadow (collides with light source quad) 
		//	  {
		//			answerT += m->Shade(ray,hit,dirToLight,lightColor,args); 
		//	  }
		//	  smoke->AddShadowSegment(shadowR, 0, shadowH.getT());
		//	 
		//  }
		//
		// answer += answerT*(1/ double(args->num_shadow_samples)); //taking average
		//}
		//
		//else
		//{
		//	answer += m->Shade(ray,hit,dirToLightCentroid,myLightColor,args); 
		//}
  //}

 // // ----------------------------------------------
  // add contribution from reflection, if the surface is shiny
   Vec3f reflectiveColor = m->getReflectiveColor();
 
  // =================================
  // ASSIGNMENT:  ADD REFLECTIVE LOGIC
  // =================================

   MTRand mt =MTRand();
 
    float c1 = (normal.Dot3(ray.getDirection())); //V*N
  //  Ray reflectedR = Ray(point,ray.getDirection() - 2*normal*c1);  //Reflective eq. in notes
	Hit reflectedH;

//	answer +=m->getReflectiveColor()*TraceRay(reflectedR,reflectedH,bounce_count - 1);
	//smoke->AddReflectedSegment(reflectedR,0,reflectedH.getT());
  
 
	  
	}

  return color; 
}


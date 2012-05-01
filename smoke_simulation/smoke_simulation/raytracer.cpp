#include "raytracer.h"
#include "vectors.h"
#include "argparser.h"
#include "smoke.h"
#include "utils.h"
#include "mesh.h"
#include "face.h"
#include "primitive.h"

std::vector<SmokeParticle*>particles;
float Volume;
bool answer;
float b =.4; //scattering coefficient
float a = .2; //absorption coefficient
float c = b+a;

// ===========================================================================
// casts a single ray through the scene geometry and finds the closest hit
bool RayTracer::CastRay(const Ray &ray, Hit &h, bool use_rasterized_patches,BoundingBox *box) const {
  bool answer = false;

  // intersect each of the quads
  for (int i = 0; i < mesh->numOriginalQuads(); i++) {
    Face *f = mesh->getOriginalQuad(i);
    if (f->intersect(ray,h,args->intersect_backfacing)) answer = true;
  }

  // intersect each of the primitives (either the patches, or the original primitives)
  if (use_rasterized_patches) {
    for (int i = 0; i < mesh->numRasterizedPrimitiveFaces(); i++) {
      Face *f = mesh->getRasterizedPrimitiveFace(i);
      if (f->intersect(ray,h,args->intersect_backfacing)) answer = true;
    }
  } else {
    int num_primitives = mesh->numPrimitives();
    for (int i = 0; i < num_primitives; i++) {
      if (mesh->getPrimitive(i)->intersect(ray,h)) answer = true;
    }
  }

  int num_primitives = smoke->oc->getParticles().size(); 
    for (int i = 0; i < num_primitives; i++) {
      if (mesh->getPrimitive(i)->intersect(ray,h)) answer = true;
    }

	std::vector<SmokeParticle *> pp = box->getParticles();
	for(int i=0;i<pp.size();i++)
	{
		//if(pp[i]->intersect(ray,h)) answer = true;
	}
  
  return answer;
}


// ===========================================================================
// ray marching
Vec3f RayTracer::Trace(const Ray &ray, Hit &h, BoundingBox *box,BoundingBox *grid) const {
	
	Vec3f color;
	int numParticles = 10;
	float  radius         = 0.2;
	float lastSmokeCont   = 0.0;
	answer                = false;
	bool out              = false; //out of main grid
	int bg                = 0;
	int count             = 0;

	Vec3f temp = Vec3f(0,0,0);
	Vec3f direction = ray.getDirection();
	Vec3f from = ray.getOrigin();

	//Create the bounding box that surrounds our path space
	Vec3f max = Vec3f((float)from.x()+ radius, (float)from.y()+ radius,(float)from.z()+ radius);
	Vec3f min = Vec3f((float)from.x()- radius, (float)from.y()- radius,(float)from.z()- radius);
	BoundingBox bb=BoundingBox(min,max);
	std::vector<SmokeParticle *> pp;
	smoke->oc->CollectParticlesInBox(bb,pp);

	// add contributions from each light
	int num_lights = mesh->getLights().size();
	for (int i = 0; i < num_lights; i++) 
	{
		Face *f = mesh->getLights()[i];
		Vec3f lightColor = f->getMaterial()->getEmittedColor() * f->getArea();
		Vec3f myLightColor;
		Vec3f lightCentroid = f->computeCentroid();
		
		/* while in inside grid, keep marching*/
		while(!out) 
		{
			for(int i=0;i<pp.size();i++)
			{
				/* if particle in radius */;
				if(ParticleInCircle(pp[i]->getPosition(), from, radius))
				{
					answer = true;		
					Material *m = pp[i]->getMaterial();
					color += m->getEmittedColor();
					Vec3f point = ray.pointAtParameter(pp[i]->getPosition().Length());

					Vec3f dirToLightCentroid = lightCentroid-point;
					dirToLightCentroid.Normalize();
  
					double distToLightCentroid = (lightCentroid-point).Length();
					myLightColor = lightColor / (M_PI*distToLightCentroid*distToLightCentroid);
	 
					if(i-1>0)
						lastSmokeCont = multipleScattering(ray,pp[i]->getPosition(),pp[i-1]->getPosition(),lastSmokeCont,distToLightCentroid,(lightCentroid-point),myLightColor);
					else
						lastSmokeCont = multipleScattering(ray,pp[i]->getPosition(),Vec3f(0,0,0),lastSmokeCont,distToLightCentroid,(lightCentroid-point),myLightColor);
			
					
					particles.push_back(pp[i]);
					smoke->hitParticles.push_back(pp[i]->getPosition());
					count++;
					//------------------------------------
					//gaussian filter
					//------------------------------------
					const float gauss_norm = 1.8790;            // normalization of gaussian
				    const float alpha = 0.918 * gauss_norm;
				    const float beta = 1.953;
				    float gau; 
					gau = alpha * (1.0 - ( 1.0 - exp( -beta * pp[i]->getPosition().Length() / (2.0*from.Length()) ) )/( 1.0 - exp(-beta) ));

					lastSmokeCont +=gau;
					color.set(lastSmokeCont+color.x(),lastSmokeCont+color.y(),lastSmokeCont+color.z());

				}
			}
			
			smoke->hitParticles.push_back(from);
			from+=direction*radius;
			// creating new boundng box in new step
			max = Vec3f((float)from.x()+ radius,(float) from.y()+ radius,(float)from.z()+ radius);
			min = Vec3f((float)from.x()- radius, (float)from.y()- radius,(float)from.z()- radius);
			bb.Set(min,max);
			pp.clear();
			smoke->oc->CollectParticlesInBox(bb,pp);
			
			//color +=Vec3f(srgb_to_linear(mesh->background_color.r()),srgb_to_linear(mesh->background_color.g()),srgb_to_linear(mesh->background_color.b()));
			//temp +=Vec3f(srgb_to_linear(mesh->background_color.r()),srgb_to_linear(mesh->background_color.g()),srgb_to_linear(mesh->background_color.b()));
			bg++;
			
			if(!ParticleInGrid(from,grid))
				out = true;
			 smoke->hitParticles.push_back(from);
		}
	}

	Volume = M_PI*pow(radius,2)*(ray.getOrigin() - from).Length();
	return color;
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

bool RayTracer::ParticleInCircle(const Vec3f pos, const Vec3f center, double radius) const
{
	if((pos-center).Length() <= radius) return true;
	return false;
}

// ===========================================================================
// does the recursive (shadow rays & recursive rays) work
 Vec3f RayTracer::TraceRay(Ray &ray1, Hit &hit, int bounce_count) const {
 		
	Vec3f color = Vec3f(0,0,0);
	smoke->hitParticles.clear();
	Vec3f from = ray1.getOrigin();
	Vec3f n = ray1.getDirection();
	
	BoundingBox *grid = new BoundingBox();
	grid->Set(smoke->getBoundingBox());
	
	bool found = true;
	n.set(n.x()/20,n.y()/20,n.z()/20);
	//while not in bbox
	bool in = false;
	int count=0;
	do
	{
		in = ParticleInGrid(from,grid);	
		from += n;
		count++;
		smoke->hitParticles.push_back(from);
		if(count ==5000)
		{
			found = false;
			break;
		}
	}while(!in);

	BoundingBox *box = smoke->oc->getCell(from.x(),from.y(),from.z());
	hit = Hit();
	Ray ray = Ray(from,n);
	
	//Ray marching	
	if(found) color = Trace(ray,hit,box,grid);
	hit = Hit();
	//cast ray to draw rest opf elements in scene
	bool intersect = CastRay(ray1,hit,false,box);
	//if non of rays hit anything, return backgroud color
	if (!answer && !intersect) return Vec3f(srgb_to_linear(mesh->background_color.r()),srgb_to_linear(mesh->background_color.g()),srgb_to_linear(mesh->background_color.b()));

	float Density = abs(particles.size()/Volume);
	color /=Density;
	
	if (intersect)
	{
		//decide what to do based on the material
		Material *m = hit.getMaterial();
		assert (m != NULL);

		// rays coming from the light source are set to white, don't bother to ray trace further.
		if (m->getEmittedColor().Length() > 0.001) 
			return Vec3f(1,1,1);
 
		Vec3f normal = hit.getNormal();
		Vec3f point = ray.pointAtParameter(hit.getT());
		Vec3f answer;

		// ----------------------------------------------
		//  start with the indirect light (ambient light)
		Vec3f diffuse_color = m->getDiffuseColor(hit.get_s(),hit.get_t());
		// the usual ray tracing hack for indirect light
		answer = diffuse_color * args->ambient_light;
	     
		// ----------------------------------------------
		// add contributions from each light that is not in shadow
		int num_lights = mesh->getLights().size(); 
		for (int i = 0; i < num_lights; i++) 
		{
			//soft shadow logic
			Vec3f answerT;
			Face *f = mesh->getLights()[i];
			Vec3f lightColor = f->getMaterial()->getEmittedColor() * f->getArea();
			Vec3f myLightColor;
			Vec3f lightCentroid = f->computeCentroid();
			Vec3f dirToLightCentroid = lightCentroid-point;
			dirToLightCentroid.Normalize();
 
			double distToLightCentroid = (lightCentroid-point).Length();
			myLightColor = lightColor / (M_PI*distToLightCentroid*distToLightCentroid);

			// ===========================================
			// ASSIGNMENT:  ADD SHADOW & SOFT SHADOW LOGIC
			// ===========================================
			/* run once shooting to middle of light*/
			if(args->num_shadow_samples == 1)
			{
				Ray shadowR (point,dirToLightCentroid);
				Hit shadowH = Hit();
				CastRay(shadowR,shadowH,false,box);

				//IF no point bw point and light : add the lighting contribution from this particular light at this point
				if (shadowH.getMaterial()->getEmittedColor().Length() > 0.001)   // Not in shadow (collides with light source quad)
					answer += m->Shade(ray,hit,dirToLightCentroid,myLightColor,args); 
				// add the lighting contribution from this particular light at this point (fix this to check for blockers between the light & this surface)
				smoke->AddShadowSegment(shadowR,0,shadowH.getT());	
			} 
			else if(args->num_shadow_samples >1)
			{
				/* multiple shadow rays for SOFT SHADOW */
				for(int j =0;j<args->num_shadow_samples;j++)
				{
					Vec3f randomP = f->RandomPoint();   //getting random point in light
					Vec3f dirToLight = randomP - point;  //dir from random point
					double dist = dirToLight.Length();
					dirToLight.Normalize();
					lightColor = f->getMaterial()->getEmittedColor() * f->getArea();
					lightColor /= M_PI*dist*dist;

					Ray shadowR (point ,dirToLight);
					Hit shadowH = Hit();
					CastRay(shadowR,shadowH,false,box);

					if (shadowH.getMaterial()->getEmittedColor().Length() > 0.001)// Not in shadow (collides with light source quad) 
						answerT += m->Shade(ray,hit,dirToLight,lightColor,args); 
					smoke->AddShadowSegment(shadowR, 0, shadowH.getT());
				}
				answer += answerT*(1/ double(args->num_shadow_samples)); //taking average
			}
			else
				answer += m->Shade(ray,hit,dirToLightCentroid,myLightColor,args); 
		} //num lights

		// ----------------------------------------------
		// add contribution from reflection, if the surface is shiny
		Vec3f reflectiveColor = m->getReflectiveColor();
		// =================================
		// ASSIGNMENT:  ADD REFLECTIVE LOGIC
		// =================================
		MTRand mt =MTRand();
		float c1 = (normal.Dot3(ray.getDirection())); //V*N
		Ray reflectedR = Ray(point,ray.getDirection() - 2*normal*c1);  //Reflective eq. in notes
		Hit reflectedH;

		//answer +=m->getReflectiveColor()*TraceRay(reflectedR,reflectedH,bounce_count - 1);
		smoke->AddReflectedSegment(reflectedR,0,reflectedH.getT());
		color+=answer;
	}
	if(color.Length() <= 0.001)
		 return Vec3f(srgb_to_linear(mesh->background_color.r()),srgb_to_linear(mesh->background_color.g()),srgb_to_linear(mesh->background_color.b()));

	return color;
}


float RayTracer::multipleScattering(const Ray &ray,Vec3f x,Vec3f x1,float lastSmokeCont,float distToLight,Vec3f dirToLight,Vec3f lightIntensity) const
{
	Vec3f e = ray.getDirection()*-1.0f;
	float cosGamma = dirToLight.Dot3(e);
	//float smokeConstant = Density;         //scattering coefficient 
	BoundingBox *bb = smoke->oc->getCell(x.x(),x.y(),x.z());

	float s1 = (x - ray.getOrigin()).Length();    //distance from particle to camera
	float s2 = distToLight;                       // distance from particle to light
	float S = s1+s2; 

	float dl = abs((x-x1).Length())*b;
	float l1 = dl;
	float l2 = bb->getLi().Length();
	float l = l1+l2; //scatterong

	//Computing blur width
	float bw = (pow(cosGamma,2)*l*pow(S,2))/(24*((1+pow(cosGamma,2)*(a/b)*pow(l,2))/12));
	bw= sqrt(bw);
	
	float L = l;//bb->getLi().Length();
	dirToLight.Normalize();
	float theta = std::acos(s1*dirToLight.Length()); //path curvature
	float C = (1/(2*pow(cosGamma,2)))*(pow(theta,2)/l);
	float p1 = (1-pow(theta,2));
	float p2 = (1+pow(theta,2)) - (2*theta*std::cos(cosGamma));
	float P = ((1-pow(theta,2)) / (4*M_PI*(pow(p2,(float)1.5))) ); //multiple scattering phase function
	
	////Weight multiple scattering contribution
	float weight = exp(-(c/b)*l)*exp(-C)*P;
	float Ltotal = lastSmokeCont + L;//*weight;
	return Ltotal;
}
	



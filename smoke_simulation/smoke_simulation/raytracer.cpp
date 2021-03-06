#include "raytracer.h"
#include "vectors.h"
#include "argparser.h"
#include "smoke.h"
#include "utils.h"
#include "mesh.h"
#include "primitive.h"

#define EPSILON 0.05
std::vector<SmokeParticle*>particles;
float Volume;
float b =.4; //scattering coefficient
float a = .2; //absorption coefficient
float c = b+a;

// ===========================================================================
// casts a single ray through the scene geometry and finds the closest hit
bool RayTracer::CastRay(const Ray &ray, Hit &h, bool use_rasterized_patches) const {
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

  //int num_primitives = smoke->oc->getParticles().size(); 
//for (int i = 0; i < num_primitives; i++) {
  //    if (mesh->getPrimitive(i)->intersect(ray,h)) answer = true;
  //  }

	/*std::vector<SmokeParticle *> pp = box->getParticles();
	for(int i=0;i<pp.size();i++)
	{
		if(pp[i]->intersect(ray,h)) answer = true;
	}*/
  
  return answer;
}

// Sort function for photons
Vec3f GLOBAL_point;
bool sortPhotons(const SmokeParticle *p1, const  SmokeParticle *p2)
{
	return (p1->getPosition() - GLOBAL_point).Length() >(p2->getPosition() - GLOBAL_point).Length();
}
// ===========================================================================
// ray marching
Vec3f RayTracer::Trace(const Ray &ray, Face *f, Vec3f end) const {
	
    Vec3f direction = ray.getDirection();
	Vec3f from = ray.getOrigin();
	float distance = (end - from).Length();
	float distance1 = (end - from).Length();
	Vec3f color           = Vec3f(0,0,0);
	int numParticles      = 8;
	float  radius         = 1;
	float width           = 0.2;
	float T               = 1;
	int steps             =  0;
	int ns = 1;
	particles.clear();
	//Create the bounding box that surrounds our path space
	Vec3f max = Vec3f((float)from.x()+ radius+width,(float) from.y()+ radius+width,(float)from.z()+ radius+width);
	Vec3f min = Vec3f((float)from.x()- radius+width, (float)from.y()- radius+width,(float)from.z()- radius+width);
	BoundingBox bb=BoundingBox(min,max);
	std::vector<SmokeParticle *> pp;
	std::vector<SmokeParticle*>pr;
	smoke->oc->CollectParticlesInBox(bb,pp);

	Vec3f lightColor = f->getMaterial()->getEmittedColor() * f->getArea();
	Vec3f myLightColor;
	Vec3f lightCentroid = f->computeCentroid();

	/* while in inside grid, keep marching*/
	while(distance >0) 
	{
		if(steps>400)
			break;
		for(int i=0;i<pp.size();i++)
		{
			/* if particle in radius */;
			//if(ParticleInCircle(pp[i]->getPosition(), from, radius,width))
			float v=CylTest_CapsFirst(from, from +direction*width, pow(width,2), pow(radius,2), pp[i]->getPosition());
			if(v!=-1)
			{
				bool pass = true;
				for(int j=0;j<particles.size();j++)	
					if(pp[i] == particles[j]) pass = false;
				if(pass)
					pr.push_back(pp[i]);
			}
		}
		if(pr.size()>= numParticles)
		{
			GLOBAL_point = from;
			std::sort(pr.begin(), pr.end(), sortPhotons);
			radius = (from - pr[numParticles - 1]->getPosition()).Length();   	// Update radius to just contain these photons

			color += Scattering(ray,pr,numParticles,from,radius,width,T,f);
			T = T*(exp(-c*(ray.getOrigin() - from).Length()))*width;

			Vec3f lightColor = f->getMaterial()->getEmittedColor() * f->getArea();
			Vec3f myLightColor;
			Vec3f lightCentroid = f->computeCentroid();
			Vec3f dirToLightCentroid = lightCentroid-from;
			dirToLightCentroid.Normalize();
			double distToLightCentroid = (lightCentroid-from).Length();
			myLightColor = lightColor / (M_PI*distToLightCentroid*distToLightCentroid);

			Ray shadowR (from,dirToLightCentroid);
			Hit shadowH = Hit();

			CastRay(shadowR,shadowH,false);
			Hit h=Hit();
			//------------------
			double t = (from.x() - ray.getOrigin().x())/ray.getDirection().x();
			Vec3f normal = ray.pointAtParameter(t) - from;
			normal.Normalize();
			h.set(t, pr[0]->getMaterial(), normal);

			//------------------
			//IF no point bw point and light : add the lighting contribution from this particular light at this point
			if (shadowH.getMaterial()->getEmittedColor().Length() > 0.00001)  { // Not in shadow (collides with light source quad)
				//color += pr[0]->getMaterial()->Shade(ray,h,dirToLightCentroid,myLightColor,args); 
				//ns++;
			}

			smoke->AddShadowSegment(shadowR,0,shadowH.getT());	

			from+=direction*width;
			distance -=width;
			steps++;
			// creating new boundng box in new step
			max = Vec3f((float)from.x()+ radius+width,(float) from.y()+ radius+width,(float)from.z()+ radius+width);
	        min = Vec3f((float)from.x()- radius+width, (float)from.y()- radius+width,(float)from.z()- radius+width);
			bb.Set(min,max);
			pp.clear();
			pr.clear();
			smoke->oc->CollectParticlesInBox(bb,pp);
			smoke->hitParticles.push_back(from);
		}
		if(pr.size()<=0)
		{
			from+=direction*width;
			distance -=width;
			steps++;
			// creating new boundng box in new step
			max = Vec3f((float)from.x()+ radius+width,(float) from.y()+ radius+width,(float)from.z()+ radius+width);
		    min = Vec3f((float)from.x()- radius+width, (float)from.y()- radius+width,(float)from.z()- radius+width);
			bb.Set(min,max);
			pp.clear();
			smoke->oc->CollectParticlesInBox(bb,pp);
			smoke->hitParticles.push_back(from);
		}
		else
		{
			pr.clear();
			radius +=.01;
			max = Vec3f((float)from.x()+ radius+width,(float) from.y()+ radius+width,(float)from.z()+ radius+width);
		    min = Vec3f((float)from.x()- radius+width, (float)from.y()- radius+width,(float)from.z()- radius+width);
			bb.Set(min,max);
			pp.clear();
			smoke->oc->CollectParticlesInBox(bb,pp);
			if(radius>1.5)
			{	
				from+=direction*width;
				distance -=width;
				steps++;
				smoke->hitParticles.push_back(from);
			}
		}	
	}
	
	return color*width;
}

bool RayTracer::ParticleInGrid(const Vec3f position,const BoundingBox *b) const
{
	if (position.x() >= b->getMin().x() - EPSILON&&
		  position.y() >= b->getMin().y() - EPSILON &&
		  position.z() >= b->getMin().z() - EPSILON&&
		  position.x() <= b->getMax().x() +EPSILON  &&
		  position.y() <= b->getMax().y() +EPSILON &&
		  position.z() <= b->getMax().z() +EPSILON )
		return true;
  return false;
}

bool RayTracer::ParticleInCircle(const Vec3f pos, const Vec3f center, float radius,float width) const
{
	if((pos-center).Length() <= radius) return true;

	return false;
}

// ===========================================================================
// does the recursive (shadow rays & recursive rays) work
 Vec3f RayTracer::TraceRay(Ray &ray1, Hit &hit, int bounce_count) const {
 		
	Vec3f colorSmoke = Vec3f(0,0,0);

	hit = Hit();
	//cast ray to draw rest opf elements in scene
	bool intersect = CastRay(ray1,hit,false);
	//if non of rays hit anything, return backgroud color
	if (!intersect ) return Vec3f(srgb_to_linear(mesh->background_color.r()),srgb_to_linear(mesh->background_color.g()),srgb_to_linear(mesh->background_color.b()));

	 Vec3f end = ray1.pointAtParameter(hit.getT());
	//decide what to do based on the material
	Material *m = hit.getMaterial();
	assert (m != NULL);

	// rays coming from the light source are set to white, don't bother to ray trace further.
	if (m->getEmittedColor().Length() > 0.001) return Vec3f(1,1,1);
 
	Vec3f normal = hit.getNormal();
	Vec3f point = ray1.pointAtParameter(hit.getT());
	Vec3f answer;

	//  start with the indirect light (ambient light)
	Vec3f diffuse_color = m->getDiffuseColor(hit.get_s(),hit.get_t());
	// the usual ray tracing hack for indirect light
	answer = diffuse_color * args->ambient_light;

	// add contributions from each light that is not in shadow
	int num_lights = mesh->getLights().size(); 
	for (int i = 0; i < num_lights; i++) 
	{
		Vec3f answerT;
		Face *f = mesh->getLights()[i];
		Vec3f lightColor = f->getMaterial()->getEmittedColor() * f->getArea();
		Vec3f myLightColor;
		Vec3f lightCentroid = f->computeCentroid();
		Vec3f dirToLightCentroid = lightCentroid-point;
		dirToLightCentroid.Normalize();
		double distToLightCentroid = (lightCentroid-point).Length();
		myLightColor = lightColor / (M_PI*distToLightCentroid*distToLightCentroid);

		//------------------------------------------------	
		BoundingBox *grid = smoke->oc->getCell();
		//grid->Set(smoke->getBoundingBox());

		Vec3f from = ray1.getOrigin();
		Vec3f n = ray1.getDirection();
		int count = 0;
		//while not in bbox
		while(!ParticleInGrid(from,grid))
		{ 
			if(count >1000)
				break;
			count++;
			from += n;
			smoke->hitParticles.push_back(from);	
		}

		//BoundingBox *box = smoke->oc->getCell(from.x(),from.y(),from.z());
		//hit = Hit();
		//Ray ray = Ray(from,n);

		//Ray marching	
		//if(found) colorSmoke = Trace(ray,hit,box,grid);

		//----------------

		//float Density = abs(particles.size()/Volume);
		//color /=Density;

		//------------------------------------------------
		//soft shadow logic

		Ray ray = Ray(from,n);	
		colorSmoke = Trace(ray,f,end);	
     //-----------------------------------------------------

		/* run once shooting to middle of light*/
		if(args->num_shadow_samples == 1)
		{
			Ray shadowR (point,dirToLightCentroid);
			Hit shadowH = Hit();
			CastRay(shadowR,shadowH,false);

			//IF no point bw point and light : add the lighting contribution from this particular light at this point
			if (shadowH.getMaterial()->getEmittedColor().Length() > 0.001)   // Not in shadow (collides with light source quad)
				 answer += m->Shade(ray1,hit,dirToLightCentroid,myLightColor,args); 
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
				CastRay(shadowR,shadowH,false);
				if (shadowH.getMaterial()->getEmittedColor().Length() > 0.001)// Not in shadow (collides with light source quad) 
					answerT += m->Shade(ray1,hit,dirToLight,lightColor,args); 
				smoke->AddShadowSegment(shadowR, 0, shadowH.getT());
			}
			answer += answerT*(1/ double(args->num_shadow_samples)); //taking average
		}
		else
			answer += m->Shade(ray1,hit,dirToLightCentroid,myLightColor,args); 
	} //num lights

	// ----------------------------------------------
	// add contribution from reflection, if the surface is shiny
	Vec3f reflectiveColor = m->getReflectiveColor();

	MTRand mt =MTRand();
	float c1 = (normal.Dot3(ray1.getDirection())); //V*N
	Ray reflectedR = Ray(point,ray1.getDirection() - 2*normal*c1);  //Reflective eq. in notes
	Hit reflectedH;

	//answer +=m->getReflectiveColor()*TraceRay(reflectedR,reflectedH,bounce_count - 1);
	smoke->AddReflectedSegment(reflectedR,0,reflectedH.getT());


	answer +=colorSmoke;
	return answer;
}


Vec3f RayTracer::Scattering(const Ray &ray,std::vector<SmokeParticle *>pr,int numParticles,Vec3f from,float radius,float width,float T,Face *f) const
{
	Vec3f lightColor = f->getMaterial()->getEmittedColor() * f->getArea();
	Vec3f myLightColor;
	Vec3f lightCentroid = f->computeCentroid();
	Vec3f dirToLightCentroid = lightCentroid-from;
	dirToLightCentroid.Normalize();
	double distToLightCentroid = (lightCentroid-from).Length();
	myLightColor = lightColor / (M_PI*distToLightCentroid*distToLightCentroid);

	//----------------------------------------------
	Vec3f L =Vec3f(0,0,0);
	Vec3f x = from;
	BoundingBox *b0 = smoke->oc->getCell(from.x(),from.y(),from.z());
	Vec3f L2 = b0->getLi();
	Vec3f L1 = Vec3f(0,0,0);
	L1+=L2;
	for(int i=0;i<numParticles;i++)
	{
		///---------------
		L1 += (pr[i]->getMaterial()->getDiffuseColor() * pr[i]->getMaterial()->getEmittedColor())*T;
		particles.push_back(pr[i]);
		smoke->hitParticles.push_back(pr[i]->getPosition());
		//------------------------------------
		//gaussian filter
		//------------------------------------
		const float gauss_norm = 5;//1.8790;            // normalization of gaussian
		const float alpha = 0.918 * gauss_norm;
		const float beta = 1.953;
		double gau; 
		//gau = alpha * (1.0 - ( 1.0 - exp( -beta * pr[i]->getPosition().Length() / (2.0*from.Length()) ) )/( 1.0 - exp(-beta) ));
		gau = exp(-((((pr[i]->getPosition()-from).Length())/(gauss_norm))*(((pr[i]->getPosition()-from).Length())/(gauss_norm)))/2.0);
		L1.set(gau+L1.x(),gau+L1.y(),gau+L1.z());
	}

	//const float wpc_norm = 1.0;
	//float tmp = (1.0f/M_PI)/( wpc_norm * from.Length());  // estimate of density
	//L1*=tmp;
	Volume = M_PI*pow(radius,2)*width;//*(ray.getOrigin() - from).Length();
	float Density = abs(numParticles/Volume);
	L1 /=Density;
	L +=L1;

	//for(int j = 0;j<mesh->numFaces();j++){
		///Face *f = mesh->getFace(j); 
		double p = 1/(4*M_PI);
	    Vec3f Lr = (f->getMaterial()->getDiffuseColor() + f->getMaterial()->getEmittedColor())*T;
		int V = 0;      //visibility (0-1)
		float H;       // geometry term
		float pdf;     //probability of distribution function
 	
		//for(int i=0;i<args->num_smoke_samples;i++)
		//{
			Vec3f x1;
			//if(i==0)
				x1 = f->computeCentroid();
			//else
			//	x1 = f->RandomPoint();

			H = (f->computeNormal().Dot3((x-x1)/(x-x1).Length()) )/ pow((x-x1).Length(),2);
			if(H < 0) H=0;

			Vec3f n = x1 - x;   // end - origin
			n.Normalize();
			Ray r = Ray(x,n);
			Hit h = Hit();
			bool intersect = f->intersect(ray,h,false);
			if(intersect)  //visible by face
			   V=1;

			pdf = (f->computeNormal().Dot3((from-x1)/(from-x1).Length()) )/ pow((from-x1).Length(),2);

			L += (p*Lr*V*H)/pdf;  //Single Scattering
			//}	
		//}

	 //L /=(mesh->numFaces());

	//IN SCATTERING
   	Vec3f t_in =from;
	Vec3f t_out;
	std::vector<SmokeParticle *>sp;
	float rad = .5;
	double T1 = 1;
	bool out = false;
	while(!out)
	{
		Vec3f max = Vec3f((float)t_in.x()+ rad,(float) t_in.y()+ rad,(float)t_in.z()+ rad);
		Vec3f min = Vec3f((float)t_in.x()- rad, (float)t_in.y()- rad,(float)t_in.z()- rad);
		BoundingBox b1=BoundingBox(min,max);
		smoke->oc->CollectParticlesInBox(b1,sp);
				
		if(sp.size()<=0){
			out = true;
			t_out = t_in;
		}
		T1 = T1*(exp(-c*(from - t_in).Length()))*rad;
		sp.clear();
		t_in+=dirToLightCentroid;
	}
	Vec3f L_d = T1*myLightColor;
	L+=b*(1/(4*M_PI))*L_d;

	return L;
}
float RayTracer::CylTest_CapsFirst( const Vec3f & pt1, const Vec3f & pt2, float lengthsq, float radius_sq, const Vec3f & testpt ) const
{
	float dx, dy, dz;	// vector d  from line segment point 1 to point 2
	float pdx, pdy, pdz;	// vector pd from point 1 to test point
	float dot, dsq;

	dx = pt2.x() - pt1.x();	// translate so pt1 is origin.  Make vector from
	dy = pt2.y() - pt1.y();     // pt1 to pt2.  Need for this is easily eliminated
	dz = pt2.z() - pt1.z();

	pdx = testpt.x() - pt1.x();		// vector from pt1 to test point.
	pdy = testpt.y() - pt1.y();
	pdz = testpt.z() - pt1.z();

	// Dot the d and pd vectors to see if point lies behind the 
	// cylinder cap at pt1.x, pt1.y, pt1.z

	dot = pdx * dx + pdy * dy + pdz * dz;

	// If dot is less than zero the point is behind the pt1 cap.
	// If greater than the cylinder axis line segment length squared
	// then the point is outside the other end cap at pt2.

	if( dot < 0.0f || dot > lengthsq )
	{
		return( -1.0f );
	}
	else 
	{
		// Point lies within the parallel caps, so find
		// distance squared from point to line, using the fact that sin^2 + cos^2 = 1
		// the dot = cos() * |d||pd|, and cross*cross = sin^2 * |d|^2 * |pd|^2
		// Carefull: '*' means mult for scalars and dotproduct for vectors
		// In short, where dist is pt distance to cyl axis: 
		// dist = sin( pd to d ) * |pd|
		// distsq = dsq = (1 - cos^2( pd to d)) * |pd|^2
		// dsq = ( 1 - (pd * d)^2 / (|pd|^2 * |d|^2) ) * |pd|^2
		// dsq = pd * pd - dot * dot / lengthsq
		//  where lengthsq is d*d or |d|^2 that is passed into this function 

		// distance squared to the cylinder axis:

		dsq = (pdx*pdx + pdy*pdy + pdz*pdz) - dot*dot/lengthsq;

		if( dsq > radius_sq )
		{
			return( -1.0f );
		}
		else
		{
			return( dsq );		// return distance squared to axis
		}
	}
}
//std::vector<SmokeParticle*>particles;
//float Volume;
//float b =.4; //scattering coefficient
//float a = .2; //absorption coefficient
//float c = b+a;
//
//// ===========================================================================
//// casts a single ray through the scene geometry and finds the closest hit
//bool RayTracer::CastRay(const Ray &ray, Hit &h, bool use_rasterized_patches) const {
//  bool answer = false;
//
//  // intersect each of the quads
//  for (int i = 0; i < mesh->numOriginalQuads(); i++) {
//    Face *f = mesh->getOriginalQuad(i);
//    if (f->intersect(ray,h,args->intersect_backfacing)) answer = true;
//  }
//
//  // intersect each of the primitives (either the patches, or the original primitives)
//  if (use_rasterized_patches) {
//    for (int i = 0; i < mesh->numRasterizedPrimitiveFaces(); i++) {
//      Face *f = mesh->getRasterizedPrimitiveFace(i);
//      if (f->intersect(ray,h,args->intersect_backfacing)) answer = true;
//    }
//  } else {
//    int num_primitives = mesh->numPrimitives();
//    for (int i = 0; i < num_primitives; i++) {
//      if (mesh->getPrimitive(i)->intersect(ray,h)) answer = true;
//    }
//  }
//
//  //int num_primitives = smoke->oc->getParticles().size(); 
////for (int i = 0; i < num_primitives; i++) {
//  //    if (mesh->getPrimitive(i)->intersect(ray,h)) answer = true;
//  //  }
//
//	//std::vector<SmokeParticle *> pp = box->getParticles();
//	//for(int i=0;i<pp.size();i++)
//	//{
//		//if(pp[i]->intersect(ray,h)) answer = true;
//	//}
//  
//  return answer;
//}
//
//// Sort function for photons
//Vec3f GLOBAL_point;
//bool sortPhotons(const SmokeParticle *p1, const  SmokeParticle *p2)
//{
//	return (p1->getPosition() - GLOBAL_point).Length() > (p2->getPosition() - GLOBAL_point).Length();
//}
//// ===========================================================================
//// ray marching
//Vec3f RayTracer::Trace(const Ray &ray, Face *f, Vec3f end) const {
//	//particles.clear();
//	//Vec3f color;
//	//int numParticles = 25;
//	//float  radius         = 0.1;
//	//float lastSmokeCont   = 0.0;
//	//answer                = false;
//	//bool out              = false; //out of main grid
//	//int bg                = 0;
//	//int count             = 0;
//	//Vec3f temp = Vec3f(0,0,0);
//	//Vec3f direction = ray.getDirection();
//	//Vec3f from = ray.getOrigin();
//
//	////Create the bounding box that surrounds our path space
//	//Vec3f max = Vec3f((float)from.x()+ radius, (float)from.y()+ radius,(float)from.z()+ radius);
//	//Vec3f min = Vec3f((float)from.x()- radius, (float)from.y()- radius,(float)from.z()- radius);
//	//BoundingBox bb=BoundingBox(min,max);
//	//std::vector<SmokeParticle *> pp;
//	//smoke->oc->CollectParticlesInBox(bb,pp);
//
//	//// add contributions from each light
//	//int num_lights = mesh->getLights().size();
//	//for (int i = 0; i < num_lights; i++) 
//	//{
//	//	Face *f = mesh->getLights()[i];
//	//	Vec3f lightColor = f->getMaterial()->getEmittedColor() * f->getArea();
//	//	Vec3f myLightColor;
//	//	Vec3f lightCentroid = f->computeCentroid();
//
//	//	bool collected = false;
//	//	/* while in inside grid, keep marching*/
//	//	while(!out)
//	//	{
//	//		for(int i=0;i<pp.size();i++)
//	//		{
//	//			/* if particle in radius */;
//	//			if(ParticleInCircle(pp[i]->getPosition(), from, radius))
//	//			{
//	//				bool pass=true;
//	//				for(int k=0;k<particles.size();k++)
//	//				{
//	//					if(particles[k]==pp[i])
//	//						pass = false;
//	//				}
//	//				if(pass && count<= numParticles)
//	//				{
//	//					count++;
//	//					answer = true;		
//	//					Material *m = pp[i]->getMaterial();
//	//					color += m->getEmittedColor();
//	//					Vec3f point = ray.pointAtParameter(pp[i]->getPosition().Length());
//
//	//					Vec3f dirToLightCentroid = lightCentroid-point;
//	//					dirToLightCentroid.Normalize();
// // 
//	//					double distToLightCentroid = (lightCentroid-point).Length();
//	//					myLightColor = lightColor / (M_PI*distToLightCentroid*distToLightCentroid);
//	// 
//	//					if(i-1>0)
//	//						lastSmokeCont = multipleScattering(ray,pp[i]->getPosition(),pp[i-1]->getPosition(),lastSmokeCont,distToLightCentroid,(lightCentroid-point),myLightColor);
//	//					else
//	//						lastSmokeCont = multipleScattering(ray,pp[i]->getPosition(),Vec3f(0,0,0),lastSmokeCont,distToLightCentroid,(lightCentroid-point),myLightColor);
//	//		
//	//				
//	//					particles.push_back(pp[i]);
//	//					smoke->hitParticles.push_back(pp[i]->getPosition());
//	//				
//	//					//------------------------------------
//	//					//gaussian filter
//	//					//------------------------------------
//	//					const float gauss_norm = 1.8790;            // normalization of gaussian
//	//					const float alpha = 0.918 * gauss_norm;
//	//					const float beta = 1.953;
//	//					float gau; 
//	//					gau = alpha * (1.0 - ( 1.0 - exp( -beta * pp[i]->getPosition().Length() / (2.0*from.Length()) ) )/( 1.0 - exp(-beta) ));
//
//	//					lastSmokeCont +=gau;
//	//					color.set(lastSmokeCont+color.x(),lastSmokeCont+color.y(),lastSmokeCont+color.z());
//	//				}
//	//			}
//	//		}
//	//		smoke->hitParticles.push_back(from);
//	//		from+=direction*radius;
//	//		// creating new boundng box in new step
//	//		max = Vec3f((float)from.x()+ radius,(float) from.y()+ radius,(float)from.z()+ radius);
//	//		min = Vec3f((float)from.x()- radius, (float)from.y()- radius,(float)from.z()- radius);
//	//		bb.Set(min,max);
//	//		pp.clear();
//	//		smoke->oc->CollectParticlesInBox(bb,pp);
//	//		
//	//		//color +=Vec3f(srgb_to_linear(mesh->background_color.r()),srgb_to_linear(mesh->background_color.g()),srgb_to_linear(mesh->background_color.b()));
//	//		//temp +=Vec3f(srgb_to_linear(mesh->background_color.r()),srgb_to_linear(mesh->background_color.g()),srgb_to_linear(mesh->background_color.b()));
//	//		bg++;
//
//	//		if(!ParticleInGrid(from,grid))
//	//		{
//	//			if(particles.size()<numParticles )
//	//			{
//	//				count = 0;
//	//				answer = false;
//	//				particles.clear();
//	//				from = ray.getOrigin();
//	//				color = Vec3f(0,0,0);
//	//				particles.clear();
//	//				radius +=.01;
//	//				max = Vec3f((float)from.x()+ radius,(float) from.y()+ radius,(float)from.z()+ radius);
//	//				min = Vec3f((float)from.x()- radius, (float)from.y()- radius,(float)from.z()- radius);
//	//				bb.Set(min,max);
//	//				pp.clear();
//	//				smoke->oc->CollectParticlesInBox(bb,pp);
//	//				if(radius>1.5)
//	//					out = true;
//	//			}
//	//			else
//	//				out = true;
//	//		}
//	//	}	
//	//}
//	//Volume = M_PI*pow(radius,2)*(ray.getOrigin() - from).Length();
//	////std::cout<<particles.size()<<" " <<radius<<" "<<Volume<<std::endl;
//	//return color;
//    Vec3f direction = ray.getDirection();
//	Vec3f from = ray.getOrigin();
//	float distance = (end - from).Length();
//	float distance1 = (end - from).Length();
//	Vec3f color           = Vec3f(0,0,0);
//	int numParticles      = 8;
//	float  radius         = 0.5;
//	float width           = 0.2;
//	float T               = 1;
//	int steps             = 0;
//	
//	particles.clear();
//	//Create the bounding box that surrounds our path space
//	Vec3f max = Vec3f((float)from.x() + width, (float)from.y() + width,(float)from.z() + width);
//	Vec3f min = Vec3f((float)from.x() - width, (float)from.y() - width,(float)from.z() - width);
//	BoundingBox bb=BoundingBox(min,max);
//	std::vector<SmokeParticle *> pp;
//	std::vector<SmokeParticle*>pr;
//	smoke->oc->CollectParticlesInBox(bb,pp);
//		
//	/* while in inside grid, keep marching*/
//	while(distance >0) 
//	{
//		if(steps>400)
//			break;
//		for(int i=0;i<pp.size();i++)
//		{
//			/* if particle in radius */;
//			float v=CylTest_CapsFirst(from, from +direction*width, pow(width,2), pow(radius,2), pp[i]->getPosition());
//			if(v!=-1)
//			//if(ParticleInCircle(pp[i]->getPosition(), from, radius,width))
//			{
//				bool pass = true;
//				for(int j=0;j<particles.size();j++)	
//					if(pp[i] == particles[j]) pass = false;
//				if(pass)
//					pr.push_back(pp[i]);
//			}
//		}
//		if(pr.size()>= numParticles)
//		{
//			GLOBAL_point = from;
//			std::sort(pr.begin(), pr.end(), sortPhotons);
//			radius = (from - pr[numParticles - 1]->getPosition()).Length();   	// Update radius to just contain these photons
//		   
//			T = T*(exp(-c*(ray.getOrigin() - from).Length()))*width;
//			color += Scattering(ray,pr,numParticles,from,radius,width,T,f);
//			//------------------------------------------------
//			Vec3f lightColor = f->getMaterial()->getEmittedColor() * f->getArea();
//			Vec3f myLightColor;
//			Vec3f lightCentroid = f->computeCentroid();
//			Vec3f dirToLightCentroid = lightCentroid-from;
//			dirToLightCentroid.Normalize();
//			double distToLightCentroid = (lightCentroid-from).Length();
//			myLightColor = lightColor / (M_PI*distToLightCentroid*distToLightCentroid);
//
//			Vec3f t_in =from;
//			Vec3f t_out;
//			std::vector<SmokeParticle *>sp;
//			int rad = 1;
//			double T1 = 1;
//			bool out = false;
//			while(!out)
//			{
//				max = Vec3f((float)t_in.x()+ rad,(float) t_in.y()+ rad,(float)t_in.z()+ rad);
//				min = Vec3f((float)t_in.x()- rad, (float)t_in.y()- rad,(float)t_in.z()- rad);
//				BoundingBox b=BoundingBox(min,max);
//				smoke->oc->CollectParticlesInBox(b,sp);
//				
//				if(sp.size()<=0){
//					out = true;
//					t_out = t_in;
//				}
//				T1 = T1*(exp(-c*(from - t_in).Length()))*rad;
//				sp.clear();
//				t_in+=dirToLightCentroid;
//			}
//			Vec3f L_d = T1*myLightColor;
//			color+=b*(1/(4*M_PI))*L_d;
//
//			Ray shadowR (from,dirToLightCentroid);
//			Hit shadowH = Hit();
//			
//			CastRay(shadowR,shadowH,false);
//			Hit h=Hit();
//			//------------------
//			double t = (from.x() - ray.getOrigin().x())/ray.getDirection().x();
//			Vec3f normal = ray.pointAtParameter(t) - from;
//			normal.Normalize();
//			h.set(t, pr[0]->getMaterial(), normal);
//			//------------------
//			//IF no point bw point and light : add the lighting contribution from this particular light at this point
//			if (shadowH.getMaterial()->getEmittedColor().Length() > 0.00001)   // Not in shadow (collides with light source quad)
//				//color += pr[0]->getMaterial()->Shade(ray,h,dirToLightCentroid,myLightColor,args); 
//			smoke->AddShadowSegment(shadowR,0,shadowH.getT());	
//
//			//walk
//			from+=direction*width;
//			distance -=width;	
//			steps++;
//			// creating new boundng box in new step
//			max = Vec3f((float)from.x()+ radius+width,(float) from.y()+ radius+width,(float)from.z()+ radius+width);
//			min = Vec3f((float)from.x()- radius+width, (float)from.y()- radius+width,(float)from.z()- radius+width);
//			bb.Set(min,max);
//			pp.clear();
//			pr.clear();
//			smoke->oc->CollectParticlesInBox(bb,pp);
//			smoke->hitParticles.push_back(from);
//		}
//		if(pr.size()<=0)
//		{
//			from+=direction*width;
//			distance -=width;	
//			steps++;
//			// creating new boundng box in new step
//			max = Vec3f((float)from.x()+ radius+width,(float) from.y()+ radius+width,(float)from.z()+ radius+width);
//			min = Vec3f((float)from.x()- radius+width, (float)from.y()- radius+width,(float)from.z()- radius+width);
//			bb.Set(min,max);
//			pp.clear();
//			smoke->oc->CollectParticlesInBox(bb,pp);
//			smoke->hitParticles.push_back(from);
//		}
//		else
//		{
//			pr.clear();
//			radius +=.01;
//			max = Vec3f((float)from.x()+ radius+width,(float) from.y()+ radius+width,(float)from.z()+ radius+width);
//			min = Vec3f((float)from.x()- radius+width, (float)from.y()- radius+width,(float)from.z()- radius+width);
//			bb.Set(min,max);
//			pp.clear();
//			smoke->oc->CollectParticlesInBox(bb,pp);
//			if(radius>1.5)
//			{	
//				from+=direction*width;
//				distance -=width;
//				steps++;
//				smoke->hitParticles.push_back(from);
//			}
//		}	
//	}
//	return color/width;
//}
//

//// ===========================================================================
//// does the recursive (shadow rays & recursive rays) work
// Vec3f RayTracer::TraceRay(Ray &ray1, Hit &hit, int bounce_count) const {
// 		
//	Vec3f colorSmoke = Vec3f(0,0,0);
//	hit = Hit();
//	//cast ray to draw rest opf elements in scene
//	bool intersect = CastRay(ray1,hit,false);
//	//if non of rays hit anything, return backgroud color
//	if (!intersect ) return Vec3f(srgb_to_linear(mesh->background_color.r()),srgb_to_linear(mesh->background_color.g()),srgb_to_linear(mesh->background_color.b()));
//	
//	 Vec3f end = ray1.pointAtParameter(hit.getT());
//	//decide what to do based on the material
//	Material *m = hit.getMaterial();
//	assert (m != NULL);
//
//	// rays coming from the light source are set to white, don't bother to ray trace further.
//	if (m->getEmittedColor().Length() > 0.001) return Vec3f(1,1,1);
// 
//	Vec3f normal = hit.getNormal();
//	Vec3f point = ray1.pointAtParameter(hit.getT());
//	Vec3f answer;
//
//	//  start with the indirect light (ambient light)
//	Vec3f diffuse_color = m->getDiffuseColor(hit.get_s(),hit.get_t());
//	// the usual ray tracing hack for indirect light
//	answer = diffuse_color * args->ambient_light;
//		
//	// add contributions from each light that is not in shadow
//	int num_lights = mesh->getLights().size(); 
//	for (int i = 0; i < num_lights; i++) 
//	{
//		Vec3f answerT;
//		Face *f = mesh->getLights()[i];
//		Vec3f lightColor = f->getMaterial()->getEmittedColor() * f->getArea();
//		Vec3f myLightColor;
//		Vec3f lightCentroid = f->computeCentroid();
//		Vec3f dirToLightCentroid = lightCentroid-point;
//		dirToLightCentroid.Normalize();
//		double distToLightCentroid = (lightCentroid-point).Length();
//		myLightColor = lightColor / (M_PI*distToLightCentroid*distToLightCentroid);
//
//		//------------------------------------------------	
//		BoundingBox *grid = smoke->oc->getCell();
//		
//		Vec3f from = ray1.getOrigin();
//		Vec3f n = ray1.getDirection();
//		int count = 0;
//		//while not in bbox
//		while(!ParticleInGrid(from,grid))
//		{ 
//			if(count >1000)
//				break;
//			count++;
//			from += n;
//			smoke->hitParticles.push_back(from);	
//		}
//		//BoundingBox *box = smoke->oc->getCell(from.x(),from.y(),from.z());
//		//hit = Hit();
//		//Ray ray = Ray(from,n);
//	
//		//Ray marching	
//		//if(found) colorSmoke = Trace(ray,hit,box,grid);
//
//		//----------------
//	
//		//float Density = abs(particles.size()/Volume);
//		//color /=Density;
//
//		//------------------------------------------------
//		//soft shadow logic
//		
//		Ray ray = Ray(from,n);	
//		colorSmoke = Trace(ray,f,end);	
//     //-----------------------------------------------------
//	
//		/* run once shooting to middle of light*/
//		if(args->num_shadow_samples == 1)
//		{
//			Ray shadowR (point,dirToLightCentroid);
//			Hit shadowH = Hit();
//			CastRay(shadowR,shadowH,false);
//
//			//IF no point bw point and light : add the lighting contribution from this particular light at this point
//			if (shadowH.getMaterial()->getEmittedColor().Length() > 0.001)   // Not in shadow (collides with light source quad)
//				 answer += m->Shade(ray1,hit,dirToLightCentroid,myLightColor,args); 
//				// add the lighting contribution from this particular light at this point (fix this to check for blockers between the light & this surface)
//			smoke->AddShadowSegment(shadowR,0,shadowH.getT());	
//		} 
//		else if(args->num_shadow_samples >1)
//		{
//			/* multiple shadow rays for SOFT SHADOW */
//			for(int j =0;j<args->num_shadow_samples;j++)
//			{
//				Vec3f randomP = f->RandomPoint();   //getting random point in light
//				Vec3f dirToLight = randomP - point;  //dir from random point
//				double dist = dirToLight.Length();
//				dirToLight.Normalize();
//				lightColor = f->getMaterial()->getEmittedColor() * f->getArea();
//				lightColor /= M_PI*dist*dist;
//
//				Ray shadowR (point ,dirToLight);
//				Hit shadowH = Hit();
//				CastRay(shadowR,shadowH,false);
//				if (shadowH.getMaterial()->getEmittedColor().Length() > 0.001)// Not in shadow (collides with light source quad) 
//					answerT += m->Shade(ray1,hit,dirToLight,lightColor,args); 
//				smoke->AddShadowSegment(shadowR, 0, shadowH.getT());
//			}
//			answer += answerT*(1/ double(args->num_shadow_samples)); //taking average
//		}
//		else
//			answer += m->Shade(ray1,hit,dirToLightCentroid,myLightColor,args); 
//	} //num lights
//
//	// ----------------------------------------------
//	// add contribution from reflection, if the surface is shiny
//	Vec3f reflectiveColor = m->getReflectiveColor();
//	
//	MTRand mt =MTRand();
//	float c1 = (normal.Dot3(ray1.getDirection())); //V*N
//	Ray reflectedR = Ray(point,ray1.getDirection() - 2*normal*c1);  //Reflective eq. in notes
//	Hit reflectedH;
//
//	//answer +=m->getReflectiveColor()*TraceRay(reflectedR,reflectedH,bounce_count - 1);
//	smoke->AddReflectedSegment(reflectedR,0,reflectedH.getT());
//	
//	answer -=colorSmoke;
//	return answer;
//}
//
//
//Vec3f RayTracer::Scattering(const Ray &ray,std::vector<SmokeParticle *>pr,int numParticles,Vec3f from,float radius,float width,float T,Face *f) const
//{
//	Vec3f lightColor = f->getMaterial()->getEmittedColor() * f->getArea();
//	Vec3f myLightColor;
//	Vec3f lightCentroid = f->computeCentroid();
//	Vec3f dirToLightCentroid = lightCentroid-from;
//	dirToLightCentroid.Normalize();
//	double distToLightCentroid = (lightCentroid-from).Length();
//	myLightColor = lightColor / (M_PI*distToLightCentroid*distToLightCentroid);
//
//	//----------------------------------------------
//	Vec3f L =Vec3f(0,0,0);
//	Vec3f x = from;
//	BoundingBox *b = smoke->oc->getCell(from.x(),from.y(),from.z());
//	Vec3f L2 = b->getLi();
//	Vec3f L1 = Vec3f(0,0,0);
//
//	for(int i=0;i<numParticles;i++)
//	{
//		///---------------
//		L1 += (pr[i]->getMaterial()->getDiffuseColor() + pr[i]->getMaterial()->getEmittedColor())*T;
//		L1+=L2;
//		particles.push_back(pr[i]);
//		smoke->hitParticles.push_back(pr[i]->getPosition());
//		//------------------------------------
//		//gaussian filter
//		//------------------------------------
//		const float gauss_norm = 1.8790;            // normalization of gaussian
//		const float alpha = 0.918 * gauss_norm;
//		const float beta = 1.953;
//		float gau; 
//		gau = alpha * (1.0 - ( 1.0 - exp( -beta * pr[i]->getPosition().Length() / (2.0*from.Length()) ) )/( 1.0 - exp(-beta) ));
//		L1.set(gau+L1.x(),gau+L1.y(),gau+L1.z());
//	}
//
//	Volume = M_PI*pow(radius,2)*width;//*(ray.getOrigin() - from).Length();
//	float Density = abs(numParticles/Volume);
//	L1 /=Density;
//	L +=L1;
//
//	for(int j = 0;j<mesh->numFaces();j++){
//		Face *f = mesh->getFace(j); 
//		double p = 1/(4*M_PI);
//	    Vec3f Lr = (f->getMaterial()->getDiffuseColor() * f->getMaterial()->getEmittedColor())*T;
//		int V = 0;      //visibility (0-1)
//		float H;       // geometry term
//		float pdf;     //probability of distribution function
// 	
//		//for(int i=0;i<args->num_smoke_samples;i++)
//		//{
//			Vec3f x1;
//			//if(i==0)
//				x1 = f->computeCentroid();
//		//	else
//			//	x1 = f->RandomPoint();
//			
//			H = (f->computeNormal().Dot3((x-x1)/(x-x1).Length()) )/ pow((x-x1).Length(),2);
//			if(H < 0) H=0;
//
//			Vec3f n = x1 - x;   // end - origin
//			n.Normalize();
//			Ray r = Ray(x,n);
//			Hit h = Hit();
//			bool intersect = f->intersect(ray,h,false);
//			if(intersect)  //visible by face
//			   V=1;
//
//			pdf = (f->computeNormal().Dot3((from-x1)/(from-x1).Length()) )/ pow((from-x1).Length(),2);
//
//			L += (p*Lr*V*H)/pdf;  //Single Scattering
//			//}	
//		}
//
//	L /=(mesh->numFaces());
//
//	return L;
//}
//	
//
//
//
//
//
////
////float RayTracer::multipleScattering(const Ray &ray,Vec3f x,Vec3f x1,float lastSmokeCont,float distToLight,Vec3f dirToLight,Vec3f lightIntensity) const
////{
////	Vec3f e = ray.getDirection()*-1.0f;
////	float cosGamma = dirToLight.Dot3(e);
////	//float smokeConstant = Density;         //scattering coefficient 
////	BoundingBox *bb = smoke->oc->getCell(x.x(),x.y(),x.z());
////
////	float s1 = (x - ray.getOrigin()).Length();    //distance from particle to camera
////	float s2 = distToLight;                       // distance from particle to light
////	float S = s1+s2; 
////
////	float dl = abs((x-x1).Length())*b;
////	float l1 = dl;
////	float l2 = bb->getLi().Length();
////	float l = l1+l2; //scatterong
////
////	//Computing blur width
////	float bw = (pow(cosGamma,2)*l*pow(S,2))/(24*((1+pow(cosGamma,2)*(a/b)*pow(l,2))/12));
////	bw= sqrt(bw);
////	
////	float L = l;//bb->getLi().Length();
////	dirToLight.Normalize();
////	float theta = std::acos(s1*dirToLight.Length()); //path curvature
////	float C = (1/(2*pow(cosGamma,2)))*(pow(theta,2)/l);
////	float p1 = (1-pow(theta,2));
////	float p2 = (1+pow(theta,2)) - (2*theta*std::cos(cosGamma));
////	float P = ((1-pow(theta,2)) / (4*M_PI*(pow(p2,(float)1.5))) ); //multiple scattering phase function
////	
////	////Weight multiple scattering contribution
////	float weight = exp(-(c/b)*l)*exp(-C)*P;
////	float Ltotal = lastSmokeCont + L;//*weight;
////	return Ltotal;
////}
//	
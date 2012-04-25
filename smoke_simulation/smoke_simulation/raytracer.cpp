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
float Density;
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
		if(pp[i]->intersect(ray,h)) answer = true;
	}
  
  return answer;
}


// ===========================================================================
// ray marching
Vec3f RayTracer::Trace(const Ray &ray, Hit &h, BoundingBox *box,BoundingBox *grid) const {
	
	Vec3f color;
	float MinimumDistance = 0.2;
	answer  = false;
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
	std::vector<SmokeParticle *> pp = mBox->getParticles();

	int num_lights = mesh->getLights().size();
	float lastSmokeCont = 0.0;
	// add contributions from each light that is not in shadow
	for (int i = 0; i < num_lights; i++) 
	{
		Face *f = mesh->getLights()[i];
		Vec3f lightColor = f->getMaterial()->getEmittedColor() * f->getArea();
		Vec3f myLightColor;
		Vec3f lightCentroid = f->computeCentroid();
			
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
				int count = 0;
				for(int i=0;i<pp.size();i++)
				{
					
					in = true;
					//float distance =pp[i]->DistanceEstimator(from);
					if(ParticleInGrid(pp[i]->getPosition(),bbBox))
					{
						answer = true;		

						Material *m = pp[i]->getMaterial();
						color += m->getEmittedColor();
						Vec3f normal = pp[i]->getPosition();
						normal.Normalize();
						Vec3f point = ray.pointAtParameter(pp[i]->getPosition().Length());

						Vec3f dirToLightCentroid = lightCentroid-point;
						dirToLightCentroid.Normalize();
  
						double distToLightCentroid = (lightCentroid-point).Length();
						myLightColor = lightColor / (M_PI*distToLightCentroid*distToLightCentroid);
	 

						if(i-1>0)
							lastSmokeCont = multipleScattering(ray,pp[i]->getPosition(),pp[i-1]->getPosition(),lastSmokeCont,distToLightCentroid,(lightCentroid-point),myLightColor);
						else
							lastSmokeCont = multipleScattering(ray,pp[i]->getPosition(),Vec3f(0,0,0),lastSmokeCont,distToLightCentroid,(lightCentroid-point),myLightColor);
			
						color.set(lastSmokeCont+color.x(),lastSmokeCont+color.y(),lastSmokeCont+color.z());
						particles.push_back(pp[i]);
						smoke->hitParticles.push_back(pp[i]->getPosition());
						count++;
					
					}
				}
				if(in)
				{
					//if(count>0)
					//	color /=count;
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
				smoke->hitParticles.push_back(from);
				from+=direction;
				//Create the bounding box that surrounds the sphere
				max = Vec3f((float)from.x()+ MinimumDistance,(float) from.y()+ MinimumDistance,(float)from.z()+ MinimumDistance);
				min = Vec3f((float)from.x()- MinimumDistance, (float)from.y()- MinimumDistance,(float)from.z()- MinimumDistance);
		
				bbBox->Set(min,max);
				pp.clear();

				mBox = smoke->oc->getCell(from.x(),from.y(),from.z());
				pp = mBox->getParticles();
				color +=Vec3f(srgb_to_linear(mesh->background_color.r()),srgb_to_linear(mesh->background_color.g()),srgb_to_linear(mesh->background_color.b()));
				
				
				std::cout<<"no"<<std::endl;
			}
			if(!ParticleInGrid(from,grid))
				out = true;
			 smoke->hitParticles.push_back(from);
		}
	}
	
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

// ===========================================================================
// does the recursive (shadow rays & recursive rays) work
 Vec3f RayTracer::TraceRay(Ray &ray1, Hit &hit, int bounce_count) const {
 		
	// add contributions from each light that is not in shadow
	int num_lights = mesh->getLights().size();
    Vec3f color = Vec3f(0,0,0);
	for (int i = 0; i < num_lights; i++) 
	{
		Face *f = mesh->getLights()[i];
		Vec3f lightColor = f->getMaterial()->getEmittedColor() * f->getArea();
		Vec3f lightCentroid = f->computeCentroid();
		smoke->oc->calculateTransmittanceOfBB(lightCentroid,c,lightColor);
	}

	smoke->hitParticles.clear();
	Vec3f from = ray1.getOrigin();
	Vec3f n = ray1.getDirection();
	
	bool in = false;
	//while not in bbox
	int count=0;
	BoundingBox *grid = new BoundingBox();
	grid->Set(smoke->getBoundingBox());
	
	for(int i=0;i<25;i++)
		from += n;
	// First cast a ray and see if we hit anything.
	n.set(n.x()/20,n.y()/20,n.z()/20);
	do
	{
		in = ParticleInGrid(from,grid);	
		from += n;
		count++;
	
		smoke->hitParticles.push_back(from);
		if(count ==500)
		{
			std::cout<<"no11"<<std::endl;
			return Vec3f(srgb_to_linear(mesh->background_color.r()),srgb_to_linear(mesh->background_color.g()),srgb_to_linear(mesh->background_color.b()));
		}
	}while(!in);

	BoundingBox *box = smoke->oc->getCell(from.x(),from.y(),from.z());

	hit = Hit();
	Ray ray = Ray(from,n);
	
	//Ray marching	
	color = Trace(ray,hit,box,grid);
	if (!answer) 
	{
		return Vec3f(srgb_to_linear(mesh->background_color.r()),srgb_to_linear(mesh->background_color.g()),srgb_to_linear(mesh->background_color.b()));
	}
	Density = abs(particles.size()/Volume);

	color /=Density;
	return color; 
}

float RayTracer::multipleScattering(const Ray &ray,Vec3f x,Vec3f x1,float lastSmokeCont,float distToLight,Vec3f dirToLight,Vec3f lightIntensity) const
{
	Vec3f e = ray.getDirection()*-1.0f;
	float cosGamma = dirToLight.Dot3(e);
	float smokeConstant = Density;         //scattering coefficient 
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
	
	float L = bb->getLi().Length();
	dirToLight.Normalize();
	float theta = std::acos(s1*dirToLight.Length()); //path curvature
	float C = (1/(2*pow(cosGamma,2)))*(pow(theta,2)/l);
	float p1 = (1-pow(theta,2));
	float p2 = (1+pow(theta,2)) - (2*theta*std::cos(cosGamma));
	//float P = ((1-pow(theta,2)) / (4*M_PI*(pow(p2,(float)1.5))) ); //multiple scattering phase function
	
	////Weight multiple scattering contribution
	//float weight = exp(-(c/b)*l)*exp(-C)*P;
	float Ltotal = lastSmokeCont + L;//*weight;
	return Ltotal;
}
	


void RayTracer::ComputeFormFactors() const {

  assert (particles.size() > 0);

  for (int i = 0; i < particles.size(); i++)
	{
		SmokeParticle *faceI = particles[i];  //patch i
		BoundingBox *bI = smoke->oc->getCell(faceI->getPosition().x(),faceI->getPosition().y(),faceI->getPosition().z());

		for (int j = 0; j < particles.size(); j++)
		{
			if (i != j)
			{
				SmokeParticle *faceJ = particles[j];  //patch i
				BoundingBox *bJ = smoke->oc->getCell(faceJ->getPosition().x(),faceJ->getPosition().y(),faceJ->getPosition().z());

				bJ->setFormFactor(0);
				double totalFF = 0;  //total form factor

				for (int k = 0; k <num_form_factor_samples; k++)
				{
					Vec3f pointI, pointJ;
					pointI = faceI->getCenter();
					pointJ = faceJ->getCenter();
					
					Vec3f v = pointI - pointJ; //END - ORIGIN
					double r = v.Length();
					v.Normalize();	
					double i_ang, j_ang; //angles
	
					Ray ray(pointJ, v);
					Hit hit;
					CastRay(ray, hit,true,bJ);
					if( (ray.pointAtParameter(hit.getT()).Length() - pointI.Length() <= 0.00001) && (ray.pointAtParameter(hit.getT()).Length() - pointI.Length() >= -0.00001) )
					{
						Vec3f ni = faceI->getCenter();
						ni.Normalize();
						Vec3f nj = faceJ->getCenter();
						nj.Normalize();
						i_ang = ni.Dot3(v);///(r*faceI->computeNormal().Length());
						j_ang = nj.Dot3(-v);///(r*faceJ->computeNormal().Length());
						double Aj = pow(faceJ->getRadius(),2)*M_PI*4;
						totalFF +=(i_ang *j_ang*Aj)/(M_PI*r*r);
						totalFF/=num_form_factor_samples;
					}
					///* addding soft shadows*/
					//for(int s = 0;s<args->num_shadow_samples;s++)
					//{
					//	pointI = faceI->RandomPoint();
					//	pointJ = faceJ->RandomPoint();

					//	Vec3f dirToLight = pointI - pointJ;  //end- origin
					//	r = dirToLight.Length();
					//	dirToLight.Normalize();

					//	Ray shadowR(pointJ,dirToLight);
					//	Hit shadowH = Hit();
					//	raytracer->CastRay(shadowR,shadowH,true);

					//	if( (shadowR.pointAtParameter(shadowH.getT()).Length() - pointJ.Length() <= 0.001) && (shadowR.pointAtParameter(shadowH.getT()).Length() - pointJ.Length() >= -0.001) )
					//	////if (!(ray.pointAtParameter(hit.getT()) - pointJ).Length() < (pointI - pointJ).Length() || ray.getDirection().Dot3(faceI->computeNormal()) > 0)
					//	{
					//		i_ang =faceI->computeNormal().Dot3( dirToLight);///(r*faceI->computeNormal().Length());
					//		j_ang = faceJ->computeNormal().Dot3(- dirToLight);///(r*faceJ->computeNormal().Length());
					//		double Aj = faceJ->getArea();
					//		totalFF +=(i_ang *j_ang*Aj)/(M_PI*r*r);
					//		totalFF /= (args->num_shadow_samples*num_form_factor_samples);
					//	}
					//}
				}

				//totalFF /= args->num_form_factor_samples;
				if (totalFF > 1)
					bJ->setFormFactor(1.0);
				else
					bJ->setFormFactor(totalFF);
			}
		}
	}
}

// ================================================================
// ================================================================

void RayTracer::Iterate() const {

    ComputeFormFactors();
	Vec3f *newRadiance = new Vec3f[particles.size()];

	for(int i=0;i<particles.size();i++)
	{
		SmokeParticle *faceI = particles[i]; 
		BoundingBox *bI = smoke->oc->getCell(faceI->getPosition().x(),faceI->getPosition().y(),faceI->getPosition().z());
		Vec3f e = particles[i]->getMaterial()->getEmittedColor(); // emited color by patch i
		Vec3f p = particles[i]->getMaterial()->getDiffuseColor(); //reflected color by patch i
		bI->setRadiance(Vec3f(0,0,0));

		for(int j=0;j<particles.size();j++)
		{
			SmokeParticle *faceJ = particles[j]; 
			BoundingBox *bJ = smoke->oc->getCell(faceJ->getPosition().x(),faceJ->getPosition().y(),faceJ->getPosition().z());
			if(i != j)  //not same face
			{
				newRadiance[i] +=bJ->getFormFactor()*bJ->getRadiance();  //discrete radiosity equation
			}
		}

		bI->setRadiance(e+newRadiance[i]*p);
		//what was absorbed/ undistributed....
		//setAbsorbed(i,  newRadiance[i] - e - (newRadiance[i]*p)); 	// received  - reflected
		//setUndistributed(i,getRadiance(i)*p- getAbsorbed(i));        //what we "lost"
		
	}

  // return the total light yet undistributed
  // (so we can decide when the solution has sufficiently converged)

	//findMaxUndistributed();
	//return total_undistributed;

  //return 0;
}

 void RayTracer::setArea(int i, double value) 
 {
    assert (i >= 0);
    area[i] = value; 
 }
 void RayTracer::setUndistributed(int i, Vec3f value) 
 { 
    assert (i >= 0);
    undistributed[i] = value; 
 }
void RayTracer::findMaxUndistributed()
{
}
void RayTracer::setAbsorbed(int i, Vec3f value) 
{ 
    assert (i >= 0);
    absorbed[i] = value;
}
//
//void RayTracer::ComputeFormFactors(BoundingBox *box) const {
// // assert (formfactors == NULL);
//  assert (particles.size() > 0);
//
//  for (int i = 0; i < particles.size(); i++)
//	{
//		SmokeParticle *faceI = particles[i];  //patch i
//
//		for (int j = 0; j < particles.size(); j++)
//		{
//			if (i != j)
//			{
//				SmokeParticle *faceJ = particles[j];  //patch i
//				box->setFormFactor(0);
//				double totalFF = 0;  //total form factor
//
//				for (int k = 0; k <num_form_factor_samples; k++)
//				{
//					Vec3f pointI, pointJ;
//					pointI = faceI->getCenter();
//					pointJ = faceJ->getCenter();
//					
//					Vec3f v = pointI - pointJ; //END - ORIGIN
//					double r = v.Length();
//					v.Normalize();	
//					double i_ang, j_ang; //angles
//	
//					Ray ray(pointJ, v);
//					Hit hit;
//					CastRay(ray, hit,true,box);
//
//					if( (ray.pointAtParameter(hit.getT()).Length() - pointI.Length() <= 0.001) && (ray.pointAtParameter(hit.getT()).Length() - pointI.Length() >= -0.001) )
//					{
//						Vec3f ni = faceI->getCenter();
//						ni.Normalize();
//						Vec3f nj = faceJ->getCenter();
//						nj.Normalize();
//						i_ang = ni.Dot3(v);///(r*faceI->computeNormal().Length());
//						j_ang = nj.Dot3(-v);///(r*faceJ->computeNormal().Length());
//						double Aj = pow(faceJ->getRadius(),2)*M_PI*4;
//						totalFF +=(i_ang *j_ang*Aj)/(M_PI*r*r);
//						totalFF/=num_form_factor_samples;
//					}
//					///* addding soft shadows*/
//					//for(int s = 0;s<args->num_shadow_samples;s++)
//					//{
//					//	pointI = faceI->RandomPoint();
//					//	pointJ = faceJ->RandomPoint();
//
//					//	Vec3f dirToLight = pointI - pointJ;  //end- origin
//					//	r = dirToLight.Length();
//					//	dirToLight.Normalize();
//
//					//	Ray shadowR(pointJ,dirToLight);
//					//	Hit shadowH = Hit();
//					//	raytracer->CastRay(shadowR,shadowH,true);
//
//					//	if( (shadowR.pointAtParameter(shadowH.getT()).Length() - pointJ.Length() <= 0.001) && (shadowR.pointAtParameter(shadowH.getT()).Length() - pointJ.Length() >= -0.001) )
//					//	////if (!(ray.pointAtParameter(hit.getT()) - pointJ).Length() < (pointI - pointJ).Length() || ray.getDirection().Dot3(faceI->computeNormal()) > 0)
//					//	{
//					//		i_ang =faceI->computeNormal().Dot3( dirToLight);///(r*faceI->computeNormal().Length());
//					//		j_ang = faceJ->computeNormal().Dot3(- dirToLight);///(r*faceJ->computeNormal().Length());
//					//		double Aj = faceJ->getArea();
//					//		totalFF +=(i_ang *j_ang*Aj)/(M_PI*r*r);
//					//		totalFF /= (args->num_shadow_samples*num_form_factor_samples);
//					//	}
//					//}
//				}
//
//				//totalFF /= args->num_form_factor_samples;
//				if (totalFF > 1)
//					box->setFormFactor(1.0);
//				else
//					box->setFormFactor(totalFF);
//			}
//		}
//	}
//}
//
//// ================================================================
//// ================================================================
//
//void RayTracer::Iterate(BoundingBox *b) const {
//    ComputeFormFactors(b);
//
//   //Setting radiance
//	if(particles.size()>0)
//	{
//		Vec3f e = particles[0]->getMaterial()->getEmittedColor(); // emited color by patch i
//		Vec3f p = particles[0]->getMaterial()->getDiffuseColor(); //reflected color by patch i
//	
//		b->setRadiance(b,e+(b->getFormFactor()*b->setRadiance()*p);   //discrete radiosity equation
//		//what was absorbed/ undistributed....
//		//setAbsorbed(i,  newRadiance[i] - e - (newRadiance[i]*p)); 	// received  - reflected
//		//setUndistributed(i,smoke->oc->getRadiance(b)*p - getAbsorbed(b));        //what we "lost"
//		
//	}
//
//  // return the total light yet undistributed
//  // (so we can decide when the solution has sufficiently converged)
//
//	//findMaxUndistributed();
//	//return total_undistributed;
//}
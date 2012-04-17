#include "smokeParticle.h"
#include "material.h"
#include "ray.h"
#include "hit.h"

bool SmokeParticle::intersect(const Ray &r, Hit &h)
{
	  // plug the explicit ray equation into the implict sphere equation and solve
		double a, b,c;
		a = 1;
		b = 2 * r.getDirection().Dot3((r.getOrigin() - center));
		c = (r.getOrigin() - center).Dot3((r.getOrigin() - center)) - radius*radius;

		bool rayI = false;
		double tDistance = -1;
		double discriminant = b*b - 4*c;
	
		if (discriminant >0) // exept two solutions
		{	
			//with discriminant
			double d = sqrt(b*b - 4*a*c);
			//solution
			// Use the smallest t value to get the collision point.
			double t1 = (-b - d)/2;
			double t2 = (-b + d)/2;
			if(t1<t2){tDistance = t1;}
			else { tDistance = t2;}
		
			rayI = true;
		}
		else if(discriminant ==0)
		{
			tDistance = (-b/(2*a));
			rayI = true;
		}
		if(tDistance < 0 ){return false;}
		if(rayI)
		{
			//updating hit
			if(h.getT() >tDistance || Hit().getT() == h.getT() )
			{
				Vec3f normal = r.pointAtParameter(tDistance) - center;
				normal.Normalize();
				h.set(tDistance, getMaterial(), normal);
				return true;
			}
		}
	  // return true if the sphere was intersected, and update the hit data structure to contain the value of t for the ray at
	  // the intersection point, the material, and the normal
	  return false;
  }

float SmokeParticle::DistanceEstimator(Vec3f p)
{
	float length = sqrt((p.x()*center.x())+(p.y()*center.y())+(p.z()*center.z()));
	return length - radius;
}
void SmokeParticle::setMaterial()
{
	center = position;
	Vec3f diffuseColor = Vec3f(0.8, 0.8, 0.8);
	Vec3f reflectiveColor = Vec3f(0,0,0);
	Vec3f emittedColor = Vec3f(.25,.25,.25);
	double roughness = 0.2;
	Material *m=new Material(diffuseColor, reflectiveColor,emittedColor,roughness);
	material = m;
}


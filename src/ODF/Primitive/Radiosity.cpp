#include "glCanvas.h"

#include "radiosity.h"
#include "mesh.h"
#include "face.h"
#include "glCanvas.h"
#include "sphere.h"
#include "raytree.h"
#include "raytracer.h"
#include "utils.h"

// ================================================================
// CONSTRUCTOR & DESTRUCTOR
// ================================================================
Radiosity::Radiosity(Mesh *m, ArgParser *a) {
  mesh = m;
  args = a;
  num_faces = -1;  
  formfactors = NULL;
  area = NULL;
  undistributed = NULL;
  absorbed = NULL;
  radiance = NULL;
  max_undistributed_patch = -1;
  total_area = -1;
  num_form_factor_samples = 10;
  Reset();
}

Radiosity::~Radiosity() {
  Cleanup();
}

void Radiosity::Cleanup() {
  delete [] formfactors;
  delete [] area;
  delete [] undistributed;
  delete [] absorbed;
  delete [] radiance;
  num_faces = -1;
  formfactors = NULL;
  area = NULL;
  undistributed = NULL;
  absorbed = NULL;
  radiance = NULL;
  max_undistributed_patch = -1;
  total_area = -1;
}

void Radiosity::Reset() {
  delete [] area;
  delete [] undistributed;
  delete [] absorbed;
  delete [] radiance;

  // create and fill the data structures
  num_faces = mesh->numFaces();
  area = new double[num_faces];
  undistributed = new Vec3f[num_faces];
  absorbed = new Vec3f[num_faces];
  radiance = new Vec3f[num_faces];
  for (int i = 0; i < num_faces; i++) {
    Face *f = mesh->getFace(i);
    f->setRadiosityPatchIndex(i);
    setArea(i,f->getArea());
    Vec3f emit = f->getMaterial()->getEmittedColor();
    setUndistributed(i,emit);
    setAbsorbed(i,Vec3f(0,0,0));
    setRadiance(i,emit);
  }

  // find the patch with the most undistributed energy
  findMaxUndistributed();
}


// =======================================================================================
// =======================================================================================

void Radiosity::findMaxUndistributed() {
  // find the patch with the most undistributed energy 
  // don't forget that the patches may have different sizes!
  max_undistributed_patch = -1;
  total_undistributed = 0;
  total_area = 0;
  double max = -1;
  for (int i = 0; i < num_faces; i++) {
    double m = getUndistributed(i).Length() * getArea(i);
    total_undistributed += m;
    total_area += getArea(i);
    if (max < m) {
      max = m;
      max_undistributed_patch = i;
    }
  }
  assert (max_undistributed_patch >= 0 && max_undistributed_patch < num_faces);
}


void Radiosity::ComputeFormFactors() {
  assert (formfactors == NULL);
  assert (num_faces > 0);
  formfactors = new double[num_faces*num_faces];



  // =====================================
  // ASSIGNMENT:  COMPUTE THE FORM FACTORS
  // =====================================
  // with help:

   for (int i = 0; i < num_faces; i++)
	{
		Face *faceI = mesh->getFace(i);  //patch i

		for (int j = 0; j < num_faces; j++)
		{
			if (i != j)
			{
				Face *faceJ = mesh->getFace(j);    //patch j
				setFormFactor(i, j, 0);
				double totalFF = 0;  //total form factor

				for (int k = 0; k <num_form_factor_samples; k++)
				{
					Vec3f pointI, pointJ;
					if (num_form_factor_samples <= 1)  //if 1, just compute center else random points
					{
						pointI = faceI->computeCentroid();
						pointJ = faceJ->computeCentroid();
					}
					else
					{
						pointI = faceI->RandomPoint();
						pointJ = faceJ->RandomPoint();
					}
					
					Vec3f v = pointI - pointJ; //END - ORIGIN
					double r = v.Length();
					v.Normalize();	
					double i_ang, j_ang; //angles
	
					Ray ray(pointJ, v);
					Hit hit;
					raytracer->CastRay(ray, hit,true);

					if( (ray.pointAtParameter(hit.getT()).Length() - pointI.Length() <= 0.001) && (ray.pointAtParameter(hit.getT()).Length() - pointI.Length() >= -0.001) )
					//if (!(ray.pointAtParameter(hit.getT()) - pointJ).Length() < (pointI - pointJ).Length() || ray.getDirection().Dot3(faceI->computeNormal()) > 0)
						//else
					{
						i_ang =faceI->computeNormal().Dot3(v);///(r*faceI->computeNormal().Length());
						j_ang = faceJ->computeNormal().Dot3(-v);///(r*faceJ->computeNormal().Length());
						double Aj = faceJ->getArea();
						totalFF +=(i_ang *j_ang*Aj)/(M_PI*r*r);
						totalFF/=num_form_factor_samples;
					}
					/* addding soft shadows*/
					for(int s = 0;s<args->num_shadow_samples;s++)
					{
						pointI = faceI->RandomPoint();
						pointJ = faceJ->RandomPoint();

						Vec3f dirToLight = pointI - pointJ;  //end- origin
						r = dirToLight.Length();
						dirToLight.Normalize();

						Ray shadowR(pointJ,dirToLight);
						Hit shadowH = Hit();
						raytracer->CastRay(shadowR,shadowH,true);

						if( (shadowR.pointAtParameter(shadowH.getT()).Length() - pointJ.Length() <= 0.001) && (shadowR.pointAtParameter(shadowH.getT()).Length() - pointJ.Length() >= -0.001) )
						////if (!(ray.pointAtParameter(hit.getT()) - pointJ).Length() < (pointI - pointJ).Length() || ray.getDirection().Dot3(faceI->computeNormal()) > 0)
						{
							i_ang =faceI->computeNormal().Dot3( dirToLight);///(r*faceI->computeNormal().Length());
							j_ang = faceJ->computeNormal().Dot3(- dirToLight);///(r*faceJ->computeNormal().Length());
							double Aj = faceJ->getArea();
							totalFF +=(i_ang *j_ang*Aj)/(M_PI*r*r);
							totalFF /= (args->num_shadow_samples*num_form_factor_samples);
						}
					}
			
				}

				//totalFF /= args->num_form_factor_samples;
				if (totalFF > 1)
					setFormFactor(i, j, 1.0);
				else
					setFormFactor(i, j, totalFF);
			}
			//normalizeFormFactors(i);
		}
	}
}

// ================================================================
// ================================================================

double Radiosity::Iterate() {
  if (formfactors == NULL) 
    ComputeFormFactors();
  assert (formfactors != NULL);

  // ==========================================
  // ASSIGNMENT:  IMPLEMENT RADIOSITY ALGORITHM
  // ==========================================

	Vec3f *newRadiance = new Vec3f[num_faces];

	for(int i=0;i<num_faces;i++)
	{
		Face *f = mesh->getFace(i);
		Vec3f e = f->getMaterial()->getEmittedColor(); // emited color by patch i
		Vec3f p = f->getMaterial()->getDiffuseColor(); //reflected color by patch i
		newRadiance[i]=Vec3f(0,0,0);

		for(int j=0;j<num_faces;j++)
		{
			if(i != j)  //not same face
			{
				newRadiance[i] +=getFormFactor(i,j)*getRadiance(j);  //discrete radiosity equation
			}
		}

		setRadiance(i,e+newRadiance[i]*p);
		//what was absorbed/ undistributed....
		setAbsorbed(i,  newRadiance[i] - e - (newRadiance[i]*p)); 	// received  - reflected
		setUndistributed(i,getRadiance(i)*p- getAbsorbed(i));        //what we "lost"
		
	}

  // return the total light yet undistributed
  // (so we can decide when the solution has sufficiently converged)

	findMaxUndistributed();
	return total_undistributed;

  //return 0;
}


// =======================================================================================
// VBO & DISPLAY FUNCTIONS
// =======================================================================================

// for interpolation
void CollectFacesWithVertex(Vertex *have, Face *f, std::vector<Face*> &faces) {
  for (unsigned int i = 0; i < faces.size(); i++) {
    if (faces[i] == f) return;
  }
  if (have != (*f)[0] && have != (*f)[1] && have != (*f)[2] && have != (*f)[3]) return;
  faces.push_back(f);
  for (int i = 0; i < 4; i++) {
    Edge *ea = f->getEdge()->getOpposite();
    Edge *eb = f->getEdge()->getNext()->getOpposite();
    Edge *ec = f->getEdge()->getNext()->getNext()->getOpposite();
    Edge *ed = f->getEdge()->getNext()->getNext()->getNext()->getOpposite();
    if (ea != NULL) CollectFacesWithVertex(have,ea->getFace(),faces);
    if (eb != NULL) CollectFacesWithVertex(have,eb->getFace(),faces);
    if (ec != NULL) CollectFacesWithVertex(have,ec->getFace(),faces);
    if (ed != NULL) CollectFacesWithVertex(have,ed->getFace(),faces);
  }
}

// different visualization modes
Vec3f Radiosity::setupHelperForColor(Face *f, int i, int j) {
  assert (mesh->getFace(i) == f);
  assert (j >= 0 && j < 4);
  if (args->render_mode == RENDER_MATERIALS) {
    return f->getMaterial()->getDiffuseColor();
  } else if (args->render_mode == RENDER_RADIANCE && args->interpolate == true) {
    std::vector<Face*> faces;
    CollectFacesWithVertex((*f)[j],f,faces);
    double total = 0;
    Vec3f color = Vec3f(0,0,0);
    Vec3f normal = f->computeNormal();
    for (unsigned int i = 0; i < faces.size(); i++) {
      Vec3f normal2 = faces[i]->computeNormal();
      double area = faces[i]->getArea();
      if (normal.Dot3(normal2) < 0.5) continue;
      assert (area > 0);
      total += area;
      color += area * getRadiance(faces[i]->getRadiosityPatchIndex());
    }
    assert (total > 0);
    color /= total;
    return color;
  } else if (args->render_mode == RENDER_LIGHTS) {
    return f->getMaterial()->getEmittedColor();
  } else if (args->render_mode == RENDER_UNDISTRIBUTED) { 
    return getUndistributed(i);
  } else if (args->render_mode == RENDER_ABSORBED) {
    return getAbsorbed(i);
  } else if (args->render_mode == RENDER_RADIANCE) {
    return getRadiance(i);
  } else if (args->render_mode == RENDER_FORM_FACTORS) {
    if (formfactors == NULL) ComputeFormFactors();
    double scale = 0.2 * total_area/getArea(i);
    double factor = scale * getFormFactor(max_undistributed_patch,i);
    return Vec3f(factor,factor,factor);
  } else {
    assert(0);
  }
  exit(0);
}



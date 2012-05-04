#include "glCanvas.h"

#include <fstream>
#include <iomanip>
#include <algorithm>

#include "smoke.h"
#include "glCanvas.h"

#include <fstream>
#include <iomanip>
#include <algorithm>

#include "smoke.h"
#include "argparser.h"
#include "boundingbox.h"
#include "vectors.h"
#include "matrix.h"
#include "marching_cubes.h"
#include "utils.h"

#define BETA_0 1.7
#define EPSILON 0.0001

// ==============================================================
// ==============================================================
// CONSTRUCTOR
// ==============================================================
// ==============================================================

Smoke::Smoke(ArgParser *_args) {
	args = _args;
	Load();
	marchingCubes = new MarchingCubes(nx+1,ny+1,nz+1,dx,dy,dz);
	SetEmptySurfaceFull();
	initializeVBOs();
	setupVBOs();
}

Smoke::~Smoke() { 
  delete [] oc; 
  delete marchingCubes; 
  cleanupVBOs(); 
}

// ==============================================================

void Smoke::Load() {    

	// open the file
	assert (args->smoke_file != "");
	std::ifstream istr(args->smoke_file.c_str());
	assert (istr != NULL);
	std::string token, token2, token3;

	// load in the grid size
	// Read in size of grid / scene, assert that it's greater than 0
	istr >> token >> nx >> ny >> nz;  assert (token=="grid");
	assert (nx > 0 && ny > 0 && nz > 0);

	// Create our original bounding box to encompass scene
	Vec3f min = Vec3f(0, 0, 0);		// Min corner is origin
	//Vec3f max = Vec3f((nx+2), (ny+2), (nz+2));
	Vec3f max = Vec3f(nx, ny, nz);	// Max corner is x,y,z
	BoundingBox * bb = new BoundingBox(min, max);

	// Initialize our octree data structure
	oc = new OCTree(bb, 0);
	grid = new BoundingBox(min - Vec3f(1,1,1), max + Vec3f(1,1,1));
	grid->set_u_plus(-1); grid->set_v_plus(-1); grid->set_w_plus(-1);

	// simulation parameters
	istr >> token >> token2;  assert (token=="flow");
	if (token2 == "compressible") compressible = true;
	else { assert (token2 == "incompressible"); compressible = false; }
	istr >> token >> token2;  assert (token=="xy_boundary");
	if (token2 == "free_slip") xy_free_slip = true;
	else { assert  (token2 == "no_slip"); xy_free_slip = false; }
	istr >> token >> token2;  assert (token=="yz_boundary");
	if (token2 == "free_slip") yz_free_slip = true;
	else { assert  (token2 == "no_slip"); yz_free_slip = false; }
	istr >> token >> token2;  assert (token=="zx_boundary");
	if (token2 == "free_slip") zx_free_slip = true;
	else { assert  (token2 == "no_slip"); zx_free_slip = false; }
	istr >> token >> viscosity;  assert (token=="viscosity");
	double gravity;
	istr >> token >> gravity;  assert (token=="gravity");
	args->gravity = Vec3f(0,-9.8,0) * gravity;
  
	// initialize marker particles 
	istr >> token >> token2 >> token3;  assert (token=="initial_particles");
	istr >> token >> density;  assert (token=="density");
	GenerateParticles(token2,token3);

	// initialize velocities
	istr >> token >> token2;  assert (token=="initial_velocity");
	if (token2 == "zero") 
	{
		std::vector<OCTree*> todo;  
		todo.push_back(oc);
		while (!todo.empty()) 
		{
			OCTree *node = todo.back();
			todo.pop_back(); 
			if (node->isLeaf()) {
				BoundingBox* c = node->getCell();
				c->set_u_plus(0);
				c->set_v_plus(0);
				c->set_w_plus(0);
			} 
			else 
			{
				// if this cell is not a leaf, explore both children
				for(int i = 0; i < 8; i++) todo.push_back(node->getChild(i));
			} 
		}
	} 
	else 
	{
		assert (token2 == "random");
		std::vector<OCTree*> todo;  
		todo.push_back(oc);
		while (!todo.empty()) 
		{
			OCTree *node = todo.back();
			todo.pop_back(); 
			if (node->isLeaf()) {
				BoundingBox* c = node->getCell();
				double dx = c->getMax().x() - c->getMin().x();
				double dy = c->getMax().y() - c->getMin().y();
				double dz = c->getMax().z() - c->getMin().y();
				double max_dim = my_max(dx,my_max(dy,dz));
				c->set_u_plus((2*args->mtrand.rand()-1)*max_dim);
				c->set_v_plus((2*args->mtrand.rand()-1)*max_dim);
				c->set_w_plus((2*args->mtrand.rand()-1)*max_dim);
			} 
			else 
			{
				// if this cell is not a leaf, explore both children
				for(int i = 0; i < 8; i++) todo.push_back(node->getChild(i));
			} 
		}
		/*
		int i,j,k;
		double max_dim = my_max(dx,my_max(dy,dz));
		for (i = -1; i <= nx; i++) 
		{
			for (j = -1; j <= ny; j++) 
			{
				for (k = -1; k <= nz; k++) 
				{
					//getBoundingBox(i,j,k)->set_u_plus((2*args->mtrand.rand()-1)*max_dim);
					//getBoundingBox(i,j,k)->set_v_plus((2*args->mtrand.rand()-1)*max_dim);
					//getBoundingBox(i,j,k)->set_w_plus((2*args->mtrand.rand()-1)*max_dim);
				}
			}
		}
		*/
	}
	// read in custom velocities
	while(istr >> token) 
	{
		double i,j,k;
		double velocity;
		assert (token == "u" || token == "v" || token == "w");
		istr >> i >> j >> k >> velocity;
		assert(i >= 0 && i < nx);
		assert(j >= 0 && j < ny);
		assert(k >= 0 && k < nz);
		if		(token == "u") oc->getCell(Vec3f(i,j,k))->set_u_plus(velocity);
		else if	(token == "v") oc->getCell(Vec3f(i,j,k))->set_v_plus(velocity);
		else if	(token == "w") oc->getCell(Vec3f(i,j,k))->set_w_plus(velocity);
		else assert(0);
		/*
		std::vector<OCTree*> todo;  
		todo.push_back(oc);
		while (!todo.empty()) 
		{
			OCTree *node = todo.back();
			todo.pop_back(); 
			if (node->isLeaf()) {
				std::cout << "Child at " << node->getCenter() << " | u: " << node->getCell()->get_u_plus() << std::endl;
			} 
			else 
			{
				// if this cell is not a leaf, explore both children
				for(int i = 0; i < 8; i++) todo.push_back(node->getChild(i));
			} 
		}
		*/
	}
	//GenerateParticles("left","random");
	SetBoundaryVelocities();
}

// ==============================================================

bool Smoke::inShape(Vec3f &pos, const std::string &shape) {
	// return true if this point is inside the "shape"
	// defined procedurally (using an implicit surface)
	if (shape == "everywhere") 
	{
		return true;
	} 
	else if (shape == "left") 
	{
		// a blob of particles on the lower left (for the dam)
		return (pos.x() < 0.2*nx && pos.y() < 0.5*ny);
	} 
	else if (shape == "drop") 
	{
		// a shallow pool of particles on the bottom
		double h = ny*dy/6.0;
		if (pos.y() < 2*h) return true;
		// and a sphere of particles above
		Vec3f center = Vec3f(nx*0.5, 5*h,nz*0.5);
		double length = (center-pos).Length();
		if (length < 0.8*h) return true;
		return false;
	} 
	else 
	{
		std::cout << "unknown shape: " << shape << std::endl;
		exit(0);
	}
}

// ==============================================================

void Smoke::GenerateParticles(const std::string &shape, const std::string &placement) {
  // create a set of points according to the "placement" token,
  // then check whether they are inside of the "shape"
	if (placement == "uniform") 
	{
		int dens = (int)pow(density,0.334);
		assert (dens*dens*dens == density);
		// the uniform grid spacing
		double spacing = 1/double(dens);
		for (double x = 0.5*spacing; x < nx; x += spacing) {
			for (double y = 0.5*spacing; y < ny; y += spacing) {
				for (double z = 0.5*spacing; z < nz; z += spacing) {
					Vec3f pos = Vec3f(x,y,z);
					if (inShape(pos,shape))
					{
						SmokeParticle *p = new SmokeParticle();
						p->setPosition(pos);
						//std::cout << p->getPosition().x() << ',' << p->getPosition().y() << ',' << p->getPosition().z() << std::endl;
						oc->AddParticle(p);
					}
				}
			}
		}
	} 
	else 
	{
		assert (placement == "random");
		// note: we don't necessarily have the same number of particles in each cell
		for (int n = 0; n < nx*ny*nz*density; n++) 
		{
			Vec3f pos = Vec3f(args->mtrand.rand()*nx, args->mtrand.rand()*ny, args->mtrand.rand()*nz);
			if (inShape(pos,shape)) 
			{      
				SmokeParticle *p = new SmokeParticle();
				p->setPosition(pos);
				//std::cout << p->getPosition().x() << ',' << p->getPosition().y() << ',' << p->getPosition().z() << std::endl;
				oc->AddParticle(p);
			}
		}
	}
}

// ==============================================================
// ==============================================================
// ANIMATION
// ==============================================================
// ==============================================================

void Smoke::Animate() {

  // the animation manager:  this is what gets done each timestep!

  ComputeNewVelocities();
  SetBoundaryVelocities();
  
  /*
  // compressible / incompressible flow
  if (compressible == false) {
    for (int iters = 0; iters < 20; iters++) {
      double max_divergence = AdjustForIncompressibility();
      SetBoundaryVelocities();
      if (max_divergence < EPSILON) break;
    }
  }
  */
  UpdatePressures();
  CopyVelocities();

  // advanced the particles through the Smoke
  MoveParticles();
  ReassignParticles();
  SetEmptySurfaceFull();

  setupVBOs();
}

// ==============================================================

void Smoke::ComputeNewVelocities() {
  double dt = args->timestep;
  int i,j,k;

  // using the formulas from Foster & Metaxas
  // NEED TO IMPLEMENT
	std::vector<OCTree*> todo;  
	todo.push_back(oc);
	// Compute new U
	while (!todo.empty()) 
	{
		OCTree *node = todo.back();
		todo.pop_back(); 
		if (node->isLeaf()) {
			Vec3f max = node->getCell()->getMax();
			Vec3f min = node->getCell()->getMin();
			double dx = max.x() - min.x(); double dy = max.y() - min.y(); double dz = max.z() - min.z();
			double i = 0.5*(min.x()+max.x()); double j = 0.5*(min.y()+max.y()); double k = 0.5*(min.z()+max.z());
			double iplus = max.x() + 0.01; double jplus = max.y() + 0.01; double kplus = max.z() + 0.01;
			double iminus = min.x() - 0.01; double jminus = min.y() - 0.01; double kminus = min.z() - 0.01;
			double dx111, dx011, dx211, dx101, dx121, dx110, dx112;
			double dy111, dy011, dy211, dy101, dy121, dy110, dy112;
			double dz111, dz011, dz211, dz101, dz121, dz110, dz112;
			BoundingBox * bb111 = node->getCell();				// i, j, k
			dx111=bb111->getMax().x()-bb111->getMin().x(); dy111=bb111->getMax().y()-bb111->getMin().y(); dz111=bb111->getMax().z()-bb111->getMin().z();
			BoundingBox * bb011 = oc->getCell(iminus, j, k);	// i-1,j, k
			dx011=bb011->getMax().x()-bb011->getMin().x(); dy011=bb011->getMax().y()-bb011->getMin().y(); dz011=bb011->getMax().z()-bb011->getMin().z();
			BoundingBox * bb211 = oc->getCell(iplus, j, k);		// i+1,j, k
			dx211=bb211->getMax().x()-bb211->getMin().x(); dy211=bb211->getMax().y()-bb211->getMin().y(); dz211=bb211->getMax().z()-bb211->getMin().z();
			BoundingBox * bb101 = oc->getCell(i, jminus, k);	// i,j-1,k
			dx101=bb101->getMax().x()-bb101->getMin().x(); dy101=bb101->getMax().y()-bb101->getMin().y(); dz101=bb101->getMax().z()-bb101->getMin().z();
			BoundingBox * bb121 = oc->getCell(i, jplus, k);		// i,j+1,k
			dx121=bb121->getMax().x()-bb121->getMin().x(); dy121=bb121->getMax().y()-bb121->getMin().y(); dz121=bb121->getMax().z()-bb121->getMin().z();
			BoundingBox * bb110 = oc->getCell(i, j, kminus);	// i,j,k-1
			dx110=bb110->getMax().x()-bb110->getMin().x(); dy110=bb110->getMax().y()-bb110->getMin().y(); dz110=bb110->getMax().z()-bb110->getMin().z();
			BoundingBox * bb112 = oc->getCell(i, j, kplus);		// i,j,k+1
			dx112=bb112->getMax().x()-bb112->getMin().x(); dy112=bb112->getMax().y()-bb112->getMin().y(); dz112=bb112->getMax().z()-bb112->getMin().z();
			if(iminus < oc->getCell()->getMin().x()) bb011 = grid; if(iplus > oc->getCell()->getMax().x()) bb211 = grid;
			if(jminus < oc->getCell()->getMin().y()) bb101 = grid; if(jplus > oc->getCell()->getMax().y()) bb121 = grid;
			if(kminus < oc->getCell()->getMin().z()) bb110 = grid; if(kplus > oc->getCell()->getMin().z()) bb112 = grid;

			//get_u_avg = 0.5*(get_u_plus(i-1,j,k)+get_u_plus(i,j,k)
			//get_uv_plus = 0.5*(get_u_plus(i,j,k) + get_u_plus(i,j+1,k)) * 0.5*(get_v_plus(i,j,k) + get_v_plus(i+1,j,k));
			double new_u_plus = bb111->get_u_plus() + dt * 
				((1/dx) * (square(0.5*(bb011->get_u_plus()*(dx111/dx011) + bb111->get_u_plus())) - square(0.5*(bb111->get_u_plus() + bb211->get_u_plus()*(dx111/dx211)))) +
				(1/dy) * ( 0.5*(bb101->get_u_plus()*(dx111/dx101) + bb111->get_u_plus()) * 0.5*(bb101->get_v_plus()*(dy111/dy101) + bb111->get_v_plus())
						 - 0.5*(bb111->get_u_plus() + bb121->get_u_plus()*(dx111/dx121)) * 0.5*(bb111->get_v_plus() + bb121->get_v_plus()*(dy111/dy121))) + 
				(1/dz) * (0.5*(bb110->get_u_plus()*(dx111/dx110) + bb111->get_u_plus()) * 0.5*(bb110->get_w_plus()*(dz111/dz110) + bb111->get_w_plus())
						 - 0.5*(bb111->get_u_plus() + bb112->get_u_plus()*(dx111/dx112)) * 0.5*(bb111->get_w_plus() + bb112->get_w_plus()*(dz111/dz112))) +
				args->gravity.x() +
				(1/dx) * (bb111->getPressure()-bb211->getPressure()*(dx111/dx211)) +
				(viscosity/square(dx)) * (bb211->get_u_plus()*(dx111/dx211) - 2*bb111->get_u_plus() + bb011->get_u_plus()*(dx111/dx011)) +
				(viscosity/square(dy)) * (bb121->get_u_plus()*(dx111/dx121) - 2*bb111->get_u_plus() + bb101->get_u_plus()*(dx111/dx101)) +
				(viscosity/square(dz)) * (bb112->get_u_plus()*(dx111/dx112) - 2*bb111->get_u_plus() + bb110->get_u_plus()*(dx111/dx110)) );
			//if(new_u_plus > 5) new_u_plus = 5.0;
			//MTRand mt = MTRand();
			//new_u_plus = 5*mt.rand();
			bb111->set_new_u_plus(new_u_plus);
		} 
		else 
		{
			// if this cell is not a leaf, explore all children
			for(int i = 0; i < 8; i++) todo.push_back(node->getChild(i));
		}
	}
	todo.clear();
	todo.push_back(oc);
	// Compute new V
	while (!todo.empty()) 
	{
		OCTree *node = todo.back();
		todo.pop_back(); 
		if (node->isLeaf()) {
			//get_u_avg = 0.5*(get_u_plus(i-1,j,k)+get_u_plus(i,j,k)
			//get_uv_plus = 0.5*(get_u_plus(i,j,k) + get_u_plus(i,j+1,k)) * 0.5*(get_v_plus(i,j,k) + get_v_plus(i+1,j,k));
			Vec3f max = node->getCell()->getMax();
			Vec3f min = node->getCell()->getMin();
			double dx = max.x() - min.x(); double dy = max.y() - min.y(); double dz = max.z() - min.z();
			double i = 0.5*(min.x()+max.x()); double j = 0.5*(min.y()+max.y()); double k = 0.5*(min.z()+max.z());
			double iplus = max.x() + 0.01; double jplus = max.y() + 0.01; double kplus = max.z() + 0.01;
			double iminus = min.x() - 0.01; double jminus = min.y() - 0.01; double kminus = min.z() - 0.01;
			double dx111, dx011, dx211, dx101, dx121, dx110, dx112;
			double dy111, dy011, dy211, dy101, dy121, dy110, dy112;
			double dz111, dz011, dz211, dz101, dz121, dz110, dz112;
			BoundingBox * bb111 = node->getCell();				// i, j, k
			dx111=bb111->getMax().x()-bb111->getMin().x(); dy111=bb111->getMax().y()-bb111->getMin().y(); dz111=bb111->getMax().z()-bb111->getMin().z();
			BoundingBox * bb011 = oc->getCell(iminus, j, k);	// i-1,j, k
			dx011=bb011->getMax().x()-bb011->getMin().x(); dy011=bb011->getMax().y()-bb011->getMin().y(); dz011=bb011->getMax().z()-bb011->getMin().z();
			BoundingBox * bb211 = oc->getCell(iplus, j, k);		// i+1,j, k
			dx211=bb211->getMax().x()-bb211->getMin().x(); dy211=bb211->getMax().y()-bb211->getMin().y(); dz211=bb211->getMax().z()-bb211->getMin().z();
			BoundingBox * bb101 = oc->getCell(i, jminus, k);	// i,j-1,k
			dx101=bb101->getMax().x()-bb101->getMin().x(); dy101=bb101->getMax().y()-bb101->getMin().y(); dz101=bb101->getMax().z()-bb101->getMin().z();
			BoundingBox * bb121 = oc->getCell(i, jplus, k);		// i,j+1,k
			dx121=bb121->getMax().x()-bb121->getMin().x(); dy121=bb121->getMax().y()-bb121->getMin().y(); dz121=bb121->getMax().z()-bb121->getMin().z();
			BoundingBox * bb110 = oc->getCell(i, j, kminus);	// i,j,k-1
			dx110=bb110->getMax().x()-bb110->getMin().x(); dy110=bb110->getMax().y()-bb110->getMin().y(); dz110=bb110->getMax().z()-bb110->getMin().z();
			BoundingBox * bb112 = oc->getCell(i, j, kplus);		// i,j,k+1
			dx112=bb112->getMax().x()-bb112->getMin().x(); dy112=bb112->getMax().y()-bb112->getMin().y(); dz112=bb112->getMax().z()-bb112->getMin().z();
			if(iminus < oc->getCell()->getMin().x()) bb011 = grid; if(iplus > oc->getCell()->getMax().x()) bb211 = grid;
			if(jminus < oc->getCell()->getMin().y()) bb101 = grid; if(jplus > oc->getCell()->getMax().y()) bb121 = grid;
			if(kminus < oc->getCell()->getMin().z()) bb110 = grid; if(kplus > oc->getCell()->getMin().z()) bb112 = grid;

			double new_v_plus = bb111->get_v_plus() + dt * 
				((1/dx) * (0.5*(bb011->get_u_plus()*(dx111/dx011) + bb111->get_u_plus()) * 0.5*(bb011->get_v_plus()*(dy111/dy011) + bb111->get_v_plus())
						 - 0.5*(bb111->get_u_plus() + bb211->get_u_plus()*(dx111/dx211)) * 0.5*(bb111->get_v_plus() + bb211->get_v_plus()*(dx111/dx211))) +
                (1/dy) * (square(0.5*(bb111->get_v_plus() + bb101->get_v_plus()*(dy111/dy101))) - square(0.5*(bb111->get_v_plus() + bb121->get_v_plus()*(dy111/dy121)))) +
                (1/dz) * (0.5*(bb110->get_v_plus()*(dy111/dy110) + bb111->get_v_plus()) * 0.5*(bb110->get_w_plus()*(dz111/dz110) + bb111->get_w_plus())
						- 0.5*(bb111->get_v_plus() + bb112->get_v_plus()*(dy111/dy112)) * 0.5*(bb111->get_w_plus() + bb112->get_w_plus()*(dz111/dz112))) +
                args->gravity.y() +
                (1/dy) * (bb111->getPressure()-bb121->getPressure()*(dy111/dy121)) +
                (viscosity/square(dx)) * (bb211->get_v_plus()*(dy111/dy211) - 2*bb111->get_v_plus() + bb011->get_v_plus()*(dy111/dy011)) +
                (viscosity/square(dy)) * (bb121->get_v_plus()*(dy111/dy121) - 2*bb111->get_v_plus() + bb101->get_v_plus()*(dy111/dy101)) +
                (viscosity/square(dz)) * (bb112->get_v_plus()*(dy111/dy112) - 2*bb111->get_v_plus() + bb110->get_v_plus()*(dy111/dy110)) );
			bb111->set_new_v_plus(new_v_plus);
			//bb111->set_new_v_plus(0);
		} 
		else 
		{
			// if this cell is not a leaf, explore all children
			for(int i = 0; i < 8; i++) todo.push_back(node->getChild(i));
		}
	}
	todo.clear();
	todo.push_back(oc);
	// Compute new W
	while (!todo.empty()) 
	{
		OCTree *node = todo.back();
		todo.pop_back(); 
		if (node->isLeaf()) {
			//get_u_avg = 0.5*(get_u_plus(i-1,j,k)+get_u_plus(i,j,k)
			//get_uv_plus = 0.5*(get_u_plus(i,j,k) + get_u_plus(i,j+1,k)) * 0.5*(get_v_plus(i,j,k) + get_v_plus(i+1,j,k));
			Vec3f max = node->getCell()->getMax();
			Vec3f min = node->getCell()->getMin();
			double dx = max.x() - min.x(); double dy = max.y() - min.y(); double dz = max.z() - min.z();
			double i = 0.5*(min.x()+max.x()); double j = 0.5*(min.y()+max.y()); double k = 0.5*(min.z()+max.z());
			double iplus = max.x() + 0.01; double jplus = max.y() + 0.01; double kplus = max.z() + 0.01;
			double iminus = min.x() - 0.01; double jminus = min.y() - 0.01; double kminus = min.z() - 0.01;
			double dx111, dx011, dx211, dx101, dx121, dx110, dx112;
			double dy111, dy011, dy211, dy101, dy121, dy110, dy112;
			double dz111, dz011, dz211, dz101, dz121, dz110, dz112;
			BoundingBox * bb111 = node->getCell();				// i, j, k
			dx111=bb111->getMax().x()-bb111->getMin().x(); dy111=bb111->getMax().y()-bb111->getMin().y(); dz111=bb111->getMax().z()-bb111->getMin().z();
			BoundingBox * bb011 = oc->getCell(iminus, j, k);	// i-1,j, k
			dx011=bb011->getMax().x()-bb011->getMin().x(); dy011=bb011->getMax().y()-bb011->getMin().y(); dz011=bb011->getMax().z()-bb011->getMin().z();
			BoundingBox * bb211 = oc->getCell(iplus, j, k);		// i+1,j, k
			dx211=bb211->getMax().x()-bb211->getMin().x(); dy211=bb211->getMax().y()-bb211->getMin().y(); dz211=bb211->getMax().z()-bb211->getMin().z();
			BoundingBox * bb101 = oc->getCell(i, jminus, k);	// i,j-1,k
			dx101=bb101->getMax().x()-bb101->getMin().x(); dy101=bb101->getMax().y()-bb101->getMin().y(); dz101=bb101->getMax().z()-bb101->getMin().z();
			BoundingBox * bb121 = oc->getCell(i, jplus, k);		// i,j+1,k
			dx121=bb121->getMax().x()-bb121->getMin().x(); dy121=bb121->getMax().y()-bb121->getMin().y(); dz121=bb121->getMax().z()-bb121->getMin().z();
			BoundingBox * bb110 = oc->getCell(i, j, kminus);	// i,j,k-1
			dx110=bb110->getMax().x()-bb110->getMin().x(); dy110=bb110->getMax().y()-bb110->getMin().y(); dz110=bb110->getMax().z()-bb110->getMin().z();
			BoundingBox * bb112 = oc->getCell(i, j, kplus);		// i,j,k+1
			dx112=bb112->getMax().x()-bb112->getMin().x(); dy112=bb112->getMax().y()-bb112->getMin().y(); dz112=bb112->getMax().z()-bb112->getMin().z();
			if(iminus < oc->getCell()->getMin().x()) bb011 = grid; if(iplus > oc->getCell()->getMax().x()) bb211 = grid;
			if(jminus < oc->getCell()->getMin().y()) bb101 = grid; if(jplus > oc->getCell()->getMax().y()) bb121 = grid;
			if(kminus < oc->getCell()->getMin().z()) bb110 = grid; if(kplus > oc->getCell()->getMin().z()) bb112 = grid;

			double new_w_plus = bb111->get_w_plus() + dt * 
				((1/dx) * (0.5*(bb011->get_u_plus()*(dx111/dx011) + bb111->get_u_plus()) * 0.5*(bb011->get_w_plus()*(dz111/dz011) + bb111->get_w_plus())
						 - 0.5*(bb111->get_u_plus() + bb211->get_u_plus()*(dx111/dx211)) * 0.5*(bb111->get_w_plus() + bb211->get_w_plus()*(dz111/dz211))) +
                (1/dy) * ( 0.5*(bb101->get_v_plus()*(dy111/dy101) + bb111->get_v_plus()) * 0.5*(bb101->get_w_plus()*(dz111/dz101) + bb111->get_w_plus())
						 - 0.5*(bb111->get_v_plus() + bb121->get_v_plus()*(dy111/dy121)) * 0.5*(bb111->get_w_plus() + bb121->get_w_plus()*(dz111/dz121))) + 
                (1/dz) * (square(0.5*(bb111->get_w_plus() + bb110->get_w_plus()*(dz111/dz110))) - square(0.5*(bb111->get_w_plus() + bb112->get_w_plus()*(dz111/dz112)))) +
                args->gravity.z() +
                (1/dz) * (bb111->getPressure()-bb112->getPressure()*(dz111/dz112)) +
                (viscosity/square(dx)) * (bb211->get_w_plus()*(dz111/dz211) - 2*bb111->get_w_plus() + bb011->get_w_plus()*(dz111/dz011)) +
                (viscosity/square(dy)) * (bb121->get_w_plus()*(dz111/dz121) - 2*bb111->get_w_plus() + bb101->get_w_plus()*(dz111/dz101)) +
                (viscosity/square(dz)) * (bb112->get_w_plus()*(dz111/dz112) - 2*bb111->get_w_plus() + bb110->get_w_plus()*(dz111/dz110)) );
			bb111->set_new_w_plus(new_w_plus);
			//bb111->set_new_w_plus(0);
		} 
		else 
		{
			// if this cell is not a leaf, explore all children
			for(int i = 0; i < 8; i++) todo.push_back(node->getChild(i));
		}
	}
  /*
  for (i = 0; i < nx-1; i++) {
    for (j = 0; j < ny; j++) {
      for (k = 0; k < nz; k++) {
        BoundingBox * bb = oc->getCell(i,j,k);
        double new_u_plus =
          get_u_plus(i,j,k) +            
          dt * ((1/dx) * (square(get_u_avg(i,j,k)) - square(get_u_avg(i+1,j,k))) +
                (1/dy) * (get_uv_plus(i,j-1,k) - get_uv_plus(i,j,k)) + 
                (1/dz) * (get_uw_plus(i,j,k-1) - get_uw_plus(i,j,k)) +
                args->gravity.x() +
                (1/dx) * (getPressure(i,j,k)-getPressure(i+1,j,k)) +
                (viscosity/square(dx)) * (get_u_plus(i+1,j  ,k  ) - 2*get_u_plus(i,j,k) + get_u_plus(i-1,j  ,k  )) +
                (viscosity/square(dy)) * (get_u_plus(i  ,j+1,k  ) - 2*get_u_plus(i,j,k) + get_u_plus(i  ,j-1,k  )) +
                (viscosity/square(dz)) * (get_u_plus(i  ,j  ,k+1) - 2*get_u_plus(i,j,k) + get_u_plus(i  ,j  ,k-1)) );
        bb->set_new_u_plus(new_u_plus);
      }
    }
  }

  for (i = 0; i < nx; i++) {
    for (j = 0; j < ny-1; j++) {
      for (k = 0; k < nz; k++) {	
        BoundingBox * bb = getBoundingBox(i,j,k);
        double new_v_plus =
          get_v_plus(i,j,k) +
          dt * ((1/dx) * (get_uv_plus(i-1,j,k) - get_uv_plus(i,j,k)) +
                (1/dy) * (square(get_v_avg(i,j,k)) - square(get_v_avg(i,j+1,k))) +
                (1/dz) * (get_vw_plus(i,j,k-1) - get_vw_plus(i,j,k)) +
                args->gravity.y() +
                (1/dy) * (getPressure(i,j,k)-getPressure(i,j+1,k)) +
                (viscosity/square(dx)) * (get_v_plus(i+1,j  ,k  ) - 2*get_v_plus(i,j,k) + get_v_plus(i-1,j  ,k  )) +
                (viscosity/square(dy)) * (get_v_plus(i  ,j+1,k  ) - 2*get_v_plus(i,j,k) + get_v_plus(i  ,j-1,k  )) +
                (viscosity/square(dz)) * (get_v_plus(i  ,j  ,k+1) - 2*get_v_plus(i,j,k) + get_v_plus(i  ,j  ,k-1)) );
        bb->set_new_v_plus(new_v_plus);
      }
    }
  }

  for (i = 0; i < nx; i++) {
    for (j = 0; j < ny; j++) {
      for (k = 0; k < nz-1; k++) {
        BoundingBox *bb = getBoundingBox(i,j,k);
        double new_w_plus =
          get_w_plus(i,j,k) +
          dt * ((1/dx) * (get_uw_plus(i-1,j,k) - get_uw_plus(i,j,k)) +
                (1/dy) * (get_vw_plus(i,j-1,k) - get_vw_plus(i,j,k)) +
                (1/dz) * (square(get_w_avg(i,j,k)) - square(get_w_avg(i,j,k+1))) +
                args->gravity.z() +
                (1/dz) * (getPressure(i,j,k)-getPressure(i,j,k+1)) +
                (viscosity/square(dx)) * (get_w_plus(i+1,j  ,k  ) - 2*get_w_plus(i,j,k) + get_w_plus(i-1,j  ,k  )) +
                (viscosity/square(dy)) * (get_w_plus(i  ,j+1,k  ) - 2*get_w_plus(i,j,k) + get_w_plus(i  ,j-1,k  )) +
                (viscosity/square(dz)) * (get_w_plus(i  ,j  ,k+1) - 2*get_w_plus(i,j,k) + get_w_plus(i  ,j  ,k-1)) );
        bb->set_new_w_plus(new_w_plus);
      }
    }
  }
  */
}


// ==============================================================

void Smoke::SetBoundaryVelocities() {

  // zero out flow perpendicular to the boundaries (no sources or sinks)
	grid->set_u_plus(0);
	grid->set_v_plus(0);
	grid->set_w_plus(0);

  // free slip or no slip boundaries (friction with boundary)
  double xy_sign = (xy_free_slip) ? 1 : -1;
  double yz_sign = (yz_free_slip) ? 1 : -1;
  double zx_sign = (zx_free_slip) ? 1 : -1;
  // NEED TO IMPLEMENT
  /*
  for (int i = 0; i < nx; i++) {
    for (int j = -1; j <= ny; j++) {
      getBoundingBox(i,j,-1)->set_u_plus(xy_sign*getBoundingBox(i,j,0)->get_u_plus());
      getBoundingBox(i,j,nz)->set_u_plus(xy_sign*getBoundingBox(i,j,nz-1)->get_u_plus());
    }
    for (int k = -1; k <= nz; k++) {
      getBoundingBox(i,-1,k)->set_u_plus(zx_sign*getBoundingBox(i,0,k)->get_u_plus());
      getBoundingBox(i,ny,k)->set_u_plus(zx_sign*getBoundingBox(i,ny-1,k)->get_u_plus());
    }
  }
  for (int j = 0; j < ny; j++) {
    for (int i = -1; i <= nx; i++) {
      getBoundingBox(i,j,-1)->set_v_plus(xy_sign*getBoundingBox(i,j,0)->get_v_plus());
      getBoundingBox(i,j,nz)->set_v_plus(xy_sign*getBoundingBox(i,j,nz-1)->get_v_plus());
    }
    for (int k = -1; k <= nz; k++) {
      getBoundingBox(-1,j,k)->set_v_plus(yz_sign*getBoundingBox(0,j,k)->get_v_plus());
      getBoundingBox(nx,j,k)->set_v_plus(yz_sign*getBoundingBox(nx-1,j,k)->get_v_plus());
    }
  }
  for (int k = 0; k < nz; k++) {
    for (int i = -1; i <= nx; i++) {
      getBoundingBox(i,-1,k)->set_w_plus(zx_sign*getBoundingBox(i,0,k)->get_w_plus());
      getBoundingBox(i,ny,k)->set_w_plus(zx_sign*getBoundingBox(i,ny-1,k)->get_w_plus());
    }
    for (int j = -1; j <= ny; j++) {
      getBoundingBox(-1,j,k)->set_w_plus(yz_sign*getBoundingBox(0,j,k)->get_w_plus());
      getBoundingBox(nx,j,k)->set_w_plus(yz_sign*getBoundingBox(nx-1,j,k)->get_w_plus());
    }
  }
  */
}

// ==============================================================

void Smoke::EmptyVelocities(BoundingBox *c) {
  
  // NEED TO IMPLEMENT
  if (c->getStatus() != CELL_EMPTY) return;
  Vec3f max = c->getMax();
  BoundingBox *ciplus = oc->getCell(max.x() + EPSILON, max.y(), max.z());
  BoundingBox *cjplus = oc->getCell(max.x(), max.y() + EPSILON, max.z());
  BoundingBox *ckplus = oc->getCell(max.x(), max.y(), max.z() + EPSILON);
  if (ciplus->getStatus() == CELL_EMPTY)
    c->set_new_u_plus(0);
  if (cjplus->getStatus() == CELL_EMPTY)
    c->set_new_v_plus(0);
  if (ckplus->getStatus() == CELL_EMPTY)
    c->set_new_w_plus(0);
}

void Smoke::CopyVelocities() {
	// NEED TO IMPLEMENT
	double dt = args->timestep;
	std::vector<OCTree*> todo;  
	todo.push_back(oc);
	while (!todo.empty()) 
	{
		OCTree *node = todo.back();
		todo.pop_back(); 
		if (node->isLeaf()) {
			BoundingBox *c = node->getCell();
			EmptyVelocities(c);
			c->copyVelocity();
			double dx = c->getMax().x() - c->getMin().x();
			double dy = c->getMax().y() - c->getMin().y();
			double dz = c->getMax().z() - c->getMin().z();
			if (fabs(c->get_u_plus()) > 0.5*dx/dt ||
				fabs(c->get_v_plus()) > 0.5*dy/dt ||
				fabs(c->get_w_plus()) > 0.5*dz/dt) {
				// velocity has exceeded reasonable threshhold
				std::cout << "velocity has exceeded reasonable threshhold, stopping animation" << std::endl;
				args->animate=false;
			}
		} 
		else 
		{
			// if this cell is not a leaf, explore all children
			for(int i = 0; i < 8; i++) todo.push_back(node->getChild(i));
		}
	}
}

// ==============================================================

double Smoke::AdjustForIncompressibility() {

  // *********************************************************************  
  // ASSIGNMENT:
  //
  // This is not a complete implementation of the Marker and BoundingBox (MAC) method.
  // Additional boundary velocities should be equalized as described in the references
  // depending on whether the boundaries are free-slip or no-slip.
  //
  // Also play around with compressible flow!
  //
  // *********************************************************************    
	double div = 0;
	double maxDiv = 0;
	// NEED TO IMPLEMENT
	/*
	for (int i = 0; i < nx; i++)  
	{
		for (int j = 0; j < ny; j++)
		{
			for (int k = 0; k < nz; k++)
			{
				/*if (getBoundingBox(i, j, k)->getStatus() == CELL_SURFACE)
				{
					if (getBoundingBox(i+1, j, k)->getStatus() == CELL_EMPTY &&
						getBoundingBox(i-1, j, k)->getStatus() != CELL_EMPTY)
					{
						adjust_new_u_plus(i+1, j, k, getBoundingBox(i-1, j, k)->get_new_u_plus());
					}
					if (getBoundingBox(i-1, j, k)->getStatus() == CELL_EMPTY &&
						getBoundingBox(i+1, j, k)->getStatus() != CELL_EMPTY )
					{
						adjust_new_u_plus(i-1, j, k, getBoundingBox(i, j, k)->get_new_u_plus());
					}
					if (getBoundingBox(i, j+1, k)->getStatus() == CELL_EMPTY &&
						getBoundingBox(i, j-1, k)->getStatus() != CELL_EMPTY)
					{
						adjust_new_v_plus(i, j+1, k, getBoundingBox(i, j-1, k)->get_new_v_plus());
					}
					if (getBoundingBox(i, j-1, k)->getStatus() == CELL_EMPTY &&
						getBoundingBox(i, j+1, k)->getStatus() != CELL_EMPTY)
					{
						adjust_new_v_plus(i, j-1, k, getBoundingBox(i, j, k)->get_new_v_plus());
					}

					int freeBoundingBoxCount = 0;
					if (getBoundingBox(i+1, j, k)->getStatus() == CELL_EMPTY)
						freeBoundingBoxCount++;
					if (getBoundingBox(i, j+1, k)->getStatus() == CELL_EMPTY)
						freeBoundingBoxCount++;
					if (getBoundingBox(i-1, j, k)->getStatus() == CELL_EMPTY)
						freeBoundingBoxCount++;
					if (getBoundingBox(i, j-1, k)->getStatus() == CELL_EMPTY)
						freeBoundingBoxCount++;

					div = - (  (get_new_u_plus(i,j,k) - get_new_u_plus(i-1,j,k)) +
								 (get_new_v_plus(i,j,k) - get_new_v_plus(i,j-1,k)) +
							    (get_new_w_plus(i,j,k) - get_new_w_plus(i,j,k-1)));

					//if (div > 0)
					//{
					//	// Push this excess volume through the free faces
					//	if (getBoundingBox(i+1, j, k)->getStatus() == CELL_EMPTY)
					//		adjust_new_u_plus(i, j, k, div/freeBoundingBoxCount);
					//	if (getBoundingBox(i, j+1, k)->getStatus() == CELL_EMPTY)
					//		adjust_new_v_plus(i, j, k, div/freeBoundingBoxCount);
					//	if (getBoundingBox(i-1, j, k)->getStatus() == CELL_EMPTY)
					//		adjust_new_u_plus(i-1, j, k, -div/freeBoundingBoxCount);
					//	if (getBoundingBox(i, j-1, k)->getStatus() == CELL_EMPTY)
					//		adjust_new_v_plus(i, j-1, k, -div/freeBoundingBoxCount);
					//}
					//else if (div < 0)
					//{
						//?
					//}
				}

				if (getBoundingBox(i, j, k)->getStatus() == CELL_FULL || getBoundingBox(i, j, k)->getStatus() == CELL_SURFACE)
				{
					//flow from cell                flow from neighbor cells (example 12-4 = 8)
					div = (get_new_u_plus(i,j,k)  - get_new_u_plus(i-1,j,k));
					div += (get_new_v_plus(i,j,k) - get_new_v_plus(i,j-1,k));
					div +=(get_new_w_plus(i,j,k)  - get_new_w_plus(i,j,k-1));
				    div =-div; //think bc adj is in flow, so flip >>in paper they have negative in equation		
  
					if(abs(div) >abs(maxDiv))
						maxDiv = div;
					
					/* Checking if we boarding all adjecent faces 
					int adjFaces = 6; //(each side cube)
					if (i == 0) //first cell
						adjFaces--;
					if (i == nx-1)  //last cell
						adjFaces--;
					if (j == 0) //first cell
						adjFaces--;
					if (j == ny-1)  // cell
						adjFaces--;
					if (k == 0)  //first cell
						adjFaces--;
					if (k == nz-1)  //last cell
						adjFaces--;
					
					//adjust velocity for face adj of in/out flow
					if (i < nx-1)
						adjust_new_u_plus(i, j, k, div/adjFaces);
					if (j < ny-1)
						adjust_new_v_plus(i, j, k, div/adjFaces);
					if (k < nz-1)
						adjust_new_v_plus(i, j, k, div/adjFaces);
					if (i-1 >= 0)
						adjust_new_u_plus(i-1, j, k, -div/adjFaces);
					if (j-1 >= 0)
						adjust_new_v_plus(i, j-1, k, -div/adjFaces);
					if (k-1 >= 0)
						adjust_new_w_plus(i, j, k-1, -div/adjFaces);
				
				}
			}

		}

	}

  // return the divergence (will be repeated while divergence > threshold)
	*/
	return fabs(maxDiv);
}

// ==============================================================

void Smoke::UpdatePressures() {
	// NEED TO IMPLEMENT
	std::vector<OCTree*> todo;  
	todo.push_back(oc);

	while (!todo.empty()) 
	{
		OCTree *node = todo.back();
		todo.pop_back(); 
		if (node->isLeaf()) {
			if (node->getCell()->getStatus() == CELL_EMPTY) 
			{
			  node->getCell()->setPressure(0);
			}
			else
			{
				BoundingBox *c = node->getCell();
				Vec3f max = c->getMax();
				Vec3f min = c->getMin();
				BoundingBox *im = oc->getCell(min.x()-EPSILON, 0.5*(max.y()+min.y()), 0.5*(max.z()+min.z()));
				BoundingBox *jm = oc->getCell(0.5*(max.x()+min.x()), min.y()-EPSILON, 0.5*(max.z()+min.z()));
				BoundingBox *km = oc->getCell(0.5*(max.x()+min.x()), 0.5*(max.y()+min.y()), min.z()-EPSILON);
				if(min.x()-EPSILON < oc->getCell()->getMin().x()) im = grid;
				if(min.y()-EPSILON < oc->getCell()->getMin().y()) jm = grid;
				if(min.z()-EPSILON < oc->getCell()->getMin().z()) km = grid;
				double dx = max.x() - min.x(); double dy = max.y() - min.y(); double dz = max.z() - min.z();
				double dxim = im->getMax().x()-im->getMin().x(); double dyjm = jm->getMax().y()-jm->getMin().y();
				double dzkm = km->getMax().z()-km->getMin().z();
				double pressure = c->getPressure();
				double divergence = 
					-( (1/dx) * (c->get_new_u_plus() - im->get_new_u_plus()*(dx/dxim)) +
					(1/dy) * (c->get_new_v_plus() - jm->get_new_v_plus()*(dy/dyjm)) +
					(1/dz) * (c->get_new_w_plus() - km->get_new_w_plus()*(dz/dzkm)) );
				double dt = args->timestep;
				double beta = BETA_0/((2*dt) * (1/square(dx) + 1/square(dy) + 1/square(dz)));
				double dp = beta*divergence;
				c->setPressure(pressure + dp);
				grid->setPressure(0);
			}
		} 
		else 
		{
			// if this cell is not a leaf, explore all children
			for(int i = 0; i < 8; i++) todo.push_back(node->getChild(i));
		}
	}
	/*
  for (int i = -1; i <= nx; i++) {
    for (int j = -1; j <= ny; j++) {
      for (int k = -1; k <= nz; k++) {
	BoundingBox *c = getBoundingBox(i,j,k);
	if (i >= 0 && i < nx && j >= 0 && j < ny && k >= 0 && k < nz) {
	  // compute divergence and increment/decrement pressure
	  double pressure = c->getPressure();
	  double divergence = 
	    - ( (1/dx) * (get_new_u_plus(i,j,k) - get_new_u_plus(i-1,j,k)) +
		(1/dy) * (get_new_v_plus(i,j,k) - get_new_v_plus(i,j-1,k)) +
		(1/dz) * (get_new_w_plus(i,j,k) - get_new_w_plus(i,j,k-1)) );
	  double dt = args->timestep;
	  double beta = BETA_0/((2*dt) * (1/square(dx) + 1/square(dy) + 1/square(dz)));
	  double dp = beta*divergence;
	  c->setPressure(pressure + dp);
	} else {
	  // zero out boundary cells (just in case)
	  c->setPressure(0);
	}

	// zero out empty cells (From Foster 2001 paper)
	if (c->getStatus() == CELL_EMPTY) {
	  c->setPressure(0);
	}
	// ========================================

      }
    }
  }
  */
}

// ==============================================================

void Smoke::MoveParticles() {
	double dt = args->timestep;
	std::vector<OCTree*> todo;  
	todo.push_back(oc);
	while (!todo.empty()) 
	{
		OCTree *node = todo.back();
		todo.pop_back(); 
		if (node->isLeaf()) {
			std::vector<SmokeParticle*> &particles = node->getParticles();
			for (unsigned int iter = 0; iter < particles.size(); iter++) {
				SmokeParticle *p = particles[iter];
				Vec3f pos = p->getPosition();
				Vec3f vel = getInterpolatedVelocity(pos);
				Vec3f pos2 = pos + vel*dt;
				// euler integration
				p->setPosition(pos2);
				/*
				if (!oc->ParticleInCell(p)) 
				{
					Vec3f pos = Vec3f(0.5*args->mtrand.rand(), 0.5 + args->mtrand.rand(ny-1), 0.5 + args->mtrand.rand(nz-1));
					p->setPosition(pos);
				}
				*/
				//if (!oc->ParticleInCell(p)) p->setPosition(pos-vel*dt);
			}
		} 
		else 
		{
			// if this cell is not a leaf, explore all children
			for(int i = 0; i < 8; i++) todo.push_back(node->getChild(i));
		}
	}
}

// ==============================================================

void Smoke::ReassignParticles() {
	std::vector<OCTree*> todo;  
	todo.push_back(oc);
	while (!todo.empty()) 
	{
		OCTree *node = todo.back();
		todo.pop_back(); 
		if (node->isLeaf()) 
		{
			std::vector<SmokeParticle*> particles = node->getParticles();
			for(int i = 0; i < particles.size(); i++)
			{
				SmokeParticle *p = particles[i];
				if (!node->ParticleInCell(p))
				{
					//if(!oc->ParticleInCell(p)) p->setPosition(Vec3f(0.1,0.1,0.1));
					BoundingBox * cell = node->getCell();
					cell->removeParticle(p);
					oc->AddParticle(p);
				}
			}
		} 
		else 
		{
			// if this cell is not a leaf, explore all children
			for(int i = 0; i < 8; i++) todo.push_back(node->getChild(i));
		}
	}
	oc->cleanupTree();
}

// ==============================================================

void Smoke::SetEmptySurfaceFull() {
	// NEED TO IMPLEMENT
	std::vector<OCTree*> todo;  
	todo.push_back(oc);
	while (!todo.empty()) 
	{
		OCTree *node = todo.back();
		todo.pop_back(); 
		if (node->isLeaf()) 
		{
			BoundingBox * cell = node->getCell();
			if(cell->numParticles() == 0)	cell->setStatus(CELL_EMPTY);
			else							cell->setStatus(CELL_FULL);
		} 
		else 
		{
			// if this cell is not a leaf, explore all children
			for(int i = 0; i < 8; i++) todo.push_back(node->getChild(i));
		}
	}
	/*
  // pick out the boundary cells
  for (i = 0; i < nx; i++) {
    for (j = 0; j < ny; j++) {
      for (k = 0; k < nz; k++) {
        BoundingBox *cell = getBoundingBox(i,j,k);
        if (cell->getStatus() == CELL_FULL &&
            (getBoundingBox(i-1,j,k)->getStatus() == CELL_EMPTY ||
             getBoundingBox(i+1,j,k)->getStatus() == CELL_EMPTY ||
             getBoundingBox(i,j-1,k)->getStatus() == CELL_EMPTY ||
             getBoundingBox(i,j+1,k)->getStatus() == CELL_EMPTY ||
             getBoundingBox(i,j,k-1)->getStatus() == CELL_EMPTY ||
             getBoundingBox(i,j,k+1)->getStatus() == CELL_EMPTY)) {
          cell->setStatus(CELL_SURFACE);
        }
      }
    }
  }
  */
}

// ==============================================================

Vec3f Smoke::getInterpolatedVelocity(const Vec3f &pos) const 
{
	double u[8],v[8],w[8];		double au[8],av[8],aw[8];
	Vec3f average, max, min;	double dx, dy, dz;
	double i, ip, im, j, jp, jm, k, kp, km;
	BoundingBox * cell[8]; cell[0] = oc->getCell(pos);
	max = cell[0]->getMax(); min = cell[0]->getMin();
	dx = max.x()-min.x(); dy = max.y()-min.y(); dz = max.z()-min.z();
	i = 0.5*(max.x()+min.x()); ip = max.x() + EPSILON; im = min.x() - EPSILON;
	j = 0.5*(max.y()+min.y()); jp = max.y() + EPSILON; jm = min.y() - EPSILON;
	k = 0.5*(max.z()+min.z()); kp = max.z() + EPSILON; km = min.z() - EPSILON;
	if(ip > oc->getMax().x()) ip = im; if(jp > oc->getMax().y()) jp = jm; if(kp > oc->getMax().z()) kp = km;
	if(im < oc->getMin().x()) im = ip; if(jm < oc->getMin().y()) jm = jp; if(km < oc->getMax().z()) km = kp;
	if(pos.x() < min.x() + 0.5*dx)
	{
		if(pos.y() < min.y() + 0.5*dy)
		{
			if(pos.z() < min.z() + 0.5*dz)
			{
				cell[0] = oc->getCell(im, jm, k); cell[4] = oc->getCell(im, j, km);
				cell[1] = oc->getCell(i , jm, k); cell[5] = oc->getCell(i , j, km);
				cell[2] = oc->getCell(im, j , k); cell[6] = oc->getCell(im, j, k );
				cell[3] = oc->getCell(i , j , k); cell[7] = oc->getCell(i , j, k );
			}
			else
			{
				cell[0] = oc->getCell(im, jm, k); cell[4] = oc->getCell(im, j, k );
				cell[1] = oc->getCell(i , jm, k); cell[5] = oc->getCell(i , j, k );
				cell[2] = oc->getCell(im, j , k); cell[6] = oc->getCell(im, j, kp);
				cell[3] = oc->getCell(i , j , k); cell[7] = oc->getCell(i , j, kp);
			}
		}
		else
		{
			if(pos.z() < min.z() + 0.5*dz)
			{
				cell[0] = oc->getCell(im, j , k); cell[4] = oc->getCell(im, j, km);
				cell[1] = oc->getCell(i , j , k); cell[5] = oc->getCell(i , j, km);
				cell[2] = oc->getCell(im, jp, k); cell[6] = oc->getCell(im, j, k );
				cell[3] = oc->getCell(i , jp, k); cell[7] = oc->getCell(i , j, k );
			}
			else
			{
				cell[0] = oc->getCell(im, j , k); cell[4] = oc->getCell(im, j, k );
				cell[1] = oc->getCell(i , j , k); cell[5] = oc->getCell(i , j, k );
				cell[2] = oc->getCell(im, jp, k); cell[6] = oc->getCell(im, j, kp);
				cell[3] = oc->getCell(i , jp, k); cell[7] = oc->getCell(i , j, kp);
			}
		}
	}
	else
	{
		if(pos.y() < min.y() + 0.5*dy)
		{
			if(pos.z() < min.z() + 0.5*dz)
			{
				cell[0] = oc->getCell(i , jm, k); cell[4] = oc->getCell(im, j, km);
				cell[1] = oc->getCell(ip, jm, k); cell[5] = oc->getCell(i , j, km);
				cell[2] = oc->getCell(i , j , k); cell[6] = oc->getCell(im, j, k );
				cell[3] = oc->getCell(ip, j , k); cell[7] = oc->getCell(i , j, k );
			}
			else
			{
				cell[0] = oc->getCell(i , jm, k); cell[4] = oc->getCell(im, j, k );
				cell[1] = oc->getCell(ip, jm, k); cell[5] = oc->getCell(i , j, k );
				cell[2] = oc->getCell(i , j , k); cell[6] = oc->getCell(im, j, kp);
				cell[3] = oc->getCell(ip, j , k); cell[7] = oc->getCell(i , j, kp);
			}
		}
		else
		{
			if(pos.z() < min.z() + 0.5*dz)
			{
				cell[0] = oc->getCell(i , j , k); cell[4] = oc->getCell(im, j, km);
				cell[1] = oc->getCell(ip, j , k); cell[5] = oc->getCell(i , j, km);
				cell[2] = oc->getCell(i , jp, k); cell[6] = oc->getCell(im, j, k );
				cell[3] = oc->getCell(ip, jp, k); cell[7] = oc->getCell(i , j, k );
			}
			else
			{
				cell[0] = oc->getCell(i , j , k); cell[4] = oc->getCell(im, j, k );
				cell[1] = oc->getCell(ip, j , k); cell[5] = oc->getCell(i , j, k );
				cell[2] = oc->getCell(i , jp, k); cell[6] = oc->getCell(im, j, kp);
				cell[3] = oc->getCell(ip, jp, k); cell[7] = oc->getCell(i , j, kp);
			}
		}
	}
	// x-y / y-x
	for(int l = 0; l < 4; l++)
	{
		max = cell[l]->getMax();	min = cell[l]->getMin();
		dx = max.x() - min.x();		dy = max.y()-min.y();
		au[l] = (1 - abs(pos.x()-max.x())/dx) * (1 - abs(pos.y()-max.y())/dy);
		u[l] = cell[l]->get_u_plus();

		av[l] = (1 - abs(pos.x()-max.x())/dx) * (1 - abs(pos.y()-max.y())/dy);
		v[l] = cell[l]->get_v_plus();
	}
	// x-z
	for(int l = 4; l < 8; l++)
	{
		max = cell[l]->getMax();	min = cell[l]->getMin();
		dx = max.x() - min.x();		dz = max.z()-min.z();
		au[l] = (1 - abs(pos.x()-max.x())/dx) * (1 - abs(pos.z()-max.z())/dz);
		u[l] = cell[l]->get_u_plus();
		aw[l] = (1 - abs(pos.x()-max.x())/dx) * (1 - abs(pos.z()-max.z())/dz);
		w[l] = cell[l]->get_w_plus();
	}
	// z-y
	for(int l = 0; l < 4; l++)
	{
		max = cell[l]->getMax();	min = cell[l]->getMin();
		dz = max.z() - min.z();		dy = max.y()-min.y();
		aw[l] = (1 - abs(pos.z()-max.z())/dz) * (1 - abs(pos.y()-max.y())/dy);
		w[l] = cell[l]->get_w_plus();
	}
	// y-z
	for(int l = 4; l < 8; l++)
	{
		max = cell[l]->getMax();	min = cell[l]->getMin();
		dy = max.y() - min.y();		dz = max.z()-min.z();
		av[l] = (1 - abs(pos.y()-max.y())/dy) * (1 - abs(pos.z()-max.z())/dz);
		v[l] = cell[l]->get_v_plus();
	}
	
	MTRand mt = MTRand();
	average.setx((au[0])*u[0] + (au[1])*u[1] + (au[2])*u[2] + (au[3])*u[3] + (au[4])*u[4] + (au[5])*u[5] + (au[6])*u[6] + (au[7])*u[7]);
	average.sety((av[0])*v[0] + (av[1])*v[1] + (av[2])*v[2] + (av[3])*v[3] + (av[4])*v[4] + (av[5])*v[5] + (av[6])*v[6] + (av[7])*v[7]);
	average.setz((aw[0])*w[0] + (aw[1])*w[1] + (aw[2])*w[2] + (aw[3])*w[3] + (aw[4])*w[4] + (aw[5])*w[5] + (aw[6])*w[6] + (aw[7])*w[7]);
	//average.setx(mt.rand()*16);
	//average.sety(mt.rand()*8);
	//average.setz(mt.rand(0));

	return average;
}




